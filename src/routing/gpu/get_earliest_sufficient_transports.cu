#include "nigiri/routing/gpu/get_earliest_sufficient_transports.cuh"
#include "nigiri/routing/gpu/raptor.h"
#include "nigiri/routing/raptor/get_earliest_sufficient_transports.h"
#include "nigiri/routing/gpu/device_timetable.cuh"

#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include <thrust/copy.h>

#include <iostream>

namespace nigiri::routing::gpu {

#define CUDA_CHECK(code)                                              \
  if ((code) != cudaSuccess) {                                        \
    std::cerr << "CUDA error: " << cudaGetErrorString(code) << " at " \
              << __FILE__ << ":" << __LINE__ << std::endl;            \
    std::terminate();                                                 \
  }

device_timetable get_device_timetable(gpu_timetable const& gtt);

struct get_transports_task {
  std::uint32_t route_idx_;
  std::uint16_t stop_idx_;
  arrival_label<64> l_;
};

struct count_consumer {
  mutable int count_ = 0;
  __device__ void operator()(device_route_label<64> const&) const {
    count_++;
  }
};

__global__ void count_kernel(
    device_timetable const tt,
    get_transports_task const* tasks,
    int const num_tasks,
    int* counts) {
  int const idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_tasks) return;

  get_transports_task const task = tasks[idx];
  count_consumer consumer;

  get_earliest_sufficient_transports<64>(
      tt,
      task.l_,
      route_idx_t{task.route_idx_},
      task.stop_idx_,
      consumer);

  counts[idx] = consumer.count_;
}

struct write_consumer {
  device_route_label<64>* out_;
  mutable int offset_;
  __device__ void operator()(device_route_label<64> const& out_l) const {
    out_[offset_++] = out_l;
  }
};

__global__ void write_kernel(
    device_timetable const tt,
    get_transports_task const* tasks,
    int const num_tasks,
    int const* offsets,
    device_route_label<64>* out_labels) {
  int const idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_tasks) return;

  get_transports_task const task = tasks[idx];
  int const start_offset = offsets[idx];

  write_consumer consumer{out_labels, start_offset};

  get_earliest_sufficient_transports<64>(
      tt,
      task.l_,
      route_idx_t{task.route_idx_},
      task.stop_idx_,
      consumer);
}

std::vector<route_label<64>> get_earliest_sufficient_transports_gpu(
    timetable const& tt,
    gpu_timetable const& gtt,
    simple_flat_matrix<std::vector<arrival_label<64>>> const& M,
    size_t const k,
    std::vector<route_idx_t> const& R) {
  // 1. Flatten the inputs into tasks
  std::vector<get_transports_task> h_tasks;
  for (auto const r : R) {
    auto const seq = tt.route_location_seq_[r];
    for (std::uint16_t s = 0; s < seq.size()-1; ++s) {
      stop const s_idx = stop{seq[s]};
      location_idx_t const l = s_idx.location_idx();
      auto const& labels = M[k][to_idx(l)];
      for (auto const& lbl : labels) {
        arrival_label<64> dl{lbl.arrival_, lbl.arrival_with_transfer_, lbl.departure_, lbl.active_days_};
        h_tasks.push_back({static_cast<std::uint32_t>(to_idx(r)), s, dl});
      }
    }
  }

  if (h_tasks.empty()) {
    return {};
  }

  // 2. Upload tasks to the GPU
  thrust::device_vector<get_transports_task> d_tasks = h_tasks;
  thrust::device_vector<int> d_counts(h_tasks.size());
  std::cout << "Created " << d_tasks.size() << " tasks." << std::endl;

  // 3. Launch the counting kernel
  int const threads_per_block = 256;
  int const blocks = (h_tasks.size() + threads_per_block - 1) / threads_per_block;
  std::cout << blocks << " blocks" << std::endl;

  device_timetable const d_tt = get_device_timetable(gtt);

  count_kernel<<<blocks, threads_per_block>>>(
      d_tt,
      thrust::raw_pointer_cast(d_tasks.data()),
      static_cast<int>(d_tasks.size()),
      thrust::raw_pointer_cast(d_counts.data()));

  cudaDeviceSynchronize();
  CUDA_CHECK(cudaGetLastError());

  // 4. Exclusive scan to compute output offsets
  thrust::device_vector<int> d_offsets(h_tasks.size());
  thrust::exclusive_scan(d_counts.begin(), d_counts.end(), d_offsets.begin());

  // Read back the total count from device to host
  int last_count = 0;
  int last_offset = 0;
  cudaMemcpy(&last_count, thrust::raw_pointer_cast(d_counts.data()) + h_tasks.size() - 1, sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(&last_offset, thrust::raw_pointer_cast(d_offsets.data()) + h_tasks.size() - 1, sizeof(int), cudaMemcpyDeviceToHost);
  int const total_output_size = last_offset + last_count;
  std::cout << "Total output size computed " << total_output_size << std::endl;
  if (total_output_size == 0) {
    return {};
  }

  // Allocate output buffer on GPU
  thrust::device_vector<device_route_label<64>> d_out_labels(total_output_size);

  // 5. Launch the writing kernel
  write_kernel<<<blocks, threads_per_block>>>(
      d_tt,
      thrust::raw_pointer_cast(d_tasks.data()),
      static_cast<int>(d_tasks.size()),
      thrust::raw_pointer_cast(d_offsets.data()),
      thrust::raw_pointer_cast(d_out_labels.data()));

  cudaDeviceSynchronize();
  CUDA_CHECK(cudaGetLastError());

  // 6. Copy output back to CPU
  std::vector<device_route_label<64>> h_out_labels(total_output_size);
  thrust::copy(d_out_labels.begin(), d_out_labels.end(), h_out_labels.begin());

  // Convert back to CPU format route_label_by_value
  std::vector<route_label<64>> result(total_output_size);
  for (int i = 0; i < total_output_size; ++i) {
    result[i].departure_ = h_out_labels[i].departure_;
    result[i].transport_day_offset_ = h_out_labels[i].transport_day_offset_;
    result[i].transport_idx_ = h_out_labels[i].transport_idx_;
    result[i].active_days_ = h_out_labels[i].active_days_;
  }

  return result;
}

}  // namespace nigiri::routing::gpu
