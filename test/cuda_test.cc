#include <deque>
#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"
#include "nigiri/routing/gpu/raptor.h"
#include "nigiri/routing/gpu/get_earliest_sufficient_transports.cuh"
#include "nigiri/routing/raptor/get_earliest_sufficient_transports.h"
#include "nigiri/routing/raptor_search.h"

#include "./loader/hrd/hrd_timetable.h"

namespace ngpu = nigiri::routing::gpu;
namespace nr = nigiri::routing;

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::test_data::hrd_timetable;

#if defined(NIGIRI_CUDA)
constexpr auto const fwd_journeys = R"(
[2020-03-30 05:00, 2020-03-30 07:15]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:00]
       TO: (C, 0000003) [2020-03-30 07:15]
leg 0: (A, 0000001) [2020-03-30 05:00] -> (B, 0000002) [2020-03-30 06:00]
   0: 0000001 A...............................................                               d: 30.03 05:00 [30.03 07:00]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/300/0000002/360/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:00 [30.03 08:00]
leg 1: (B, 0000002) [2020-03-30 06:00] -> (B, 0000002) [2020-03-30 06:02]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:15] -> (C, 0000003) [2020-03-30 07:15]
   0: 0000002 B...............................................                               d: 30.03 06:15 [30.03 08:15]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/375/0000003/435/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:15 [30.03 09:15]


[2020-03-30 05:30, 2020-03-30 07:45]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:30]
       TO: (C, 0000003) [2020-03-30 07:45]
leg 0: (A, 0000001) [2020-03-30 05:30] -> (B, 0000002) [2020-03-30 06:30]
   0: 0000001 A...............................................                               d: 30.03 05:30 [30.03 07:30]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/330/0000002/390/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:30 [30.03 08:30]
leg 1: (B, 0000002) [2020-03-30 06:30] -> (B, 0000002) [2020-03-30 06:32]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:45] -> (C, 0000003) [2020-03-30 07:45]
   0: 0000002 B...............................................                               d: 30.03 06:45 [30.03 08:45]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/405/0000003/465/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:45 [30.03 09:45]


)";

TEST(nigiri_cuda, test) {
  constexpr auto const src = source_idx_t{0U};

  auto tt = timetable{};
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const gpu_tt = ngpu::gpu_timetable{tt};
  auto algo_state = ngpu::gpu_raptor_state{gpu_tt};
  auto search_state = nr::search_state{};

  auto q = routing::query{
      .start_time_ =
          interval{unixtime_t{sys_days{2020_y / March / 30}} + 5_hours,
                   unixtime_t{sys_days{2020_y / March / 30}} + 6_hours},
      .start_ = {{tt.locations_.location_id_to_idx_.at({"0000001", src}),
                  0_minutes, 0U}},
      .destination_ = {{tt.locations_.location_id_to_idx_.at({"0000003", src}),
                        0_minutes, 0U}},
      .via_stops_ = {}};
  auto const results =
      *(routing::raptor_search(tt, nullptr, search_state, algo_state,
                               std::move(q), direction::kForward)
            .journeys_);

  std::stringstream ss;
  ss << "\n";
  for (auto const& x : results) {
    x.print(ss, tt);
    ss << "\n\n";
  }
  EXPECT_EQ(std::string_view{fwd_journeys}, ss.str());
}

TEST(nigiri_cuda, get_earliest_sufficient_transports_gpu_test) {
  constexpr auto const src = source_idx_t{0U};

  auto tt = timetable{};
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const gpu_tt = ngpu::gpu_timetable{tt};

  // We set up a simple_flat_matrix with 1 row (N=1) and H = tt.n_locations()
  auto const n_locations = tt.n_locations();
  auto M = simple_flat_matrix<std::vector<routing::arrival_label<64>>>{1U, n_locations};

  // Find location "A" ("0000001")
  auto const loc_a = tt.locations_.location_id_to_idx_.at({"0000001", src});

  // Create an active days bitset (day 0 active)
  cista::bitset<64> active_days;
  active_days.set(7, true);

  // Add a label at location A
  // departure = 05:00 (300 minutes after midnight)
  // arrival = 07:00 (420 minutes after midnight)
  // arrival_with_transfer = 07:00
  routing::arrival_label<64> l{
      .arrival_ = 420,
      .arrival_with_transfer_ = 420,
      .departure_ = 300,
      .active_days_ = active_days
  };
  M[0][to_idx(loc_a)].push_back(l);

  // Print the route stations
  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    const auto stop_seq = tt.route_location_seq_[route_idx];
    std::cout << "Route " << route_idx << ": ";
    for (auto i = 0U; i < stop_seq.size(); ++i) {
      const auto stp = stop{stop_seq[i]};
      std::cout << stp.location_idx() << (i < stop_seq.size() - 1 ? "->" : "");
    }
    std::cout << "\n";
    const auto transport_range = tt.route_transport_ranges_[route_idx];
    for (const auto t : transport_range) {
      std::cout << "transport_idx " << t << ": ";
      for (auto i = 0U; i < stop_seq.size(); ++i) {
        if (i > 0) {
          const auto arr = tt.event_mam(route_idx, t, i, event_type::kArr);
          std::cout << "arr: " << arr;
        }
        if (i < stop_seq.size() - 1) {
          const auto dep = tt.event_mam(route_idx, t, i, event_type::kDep);
          std::cout << "dep: " << dep << " -> ";
        }
      }
      std::cout << " " << tt.bitfields_[tt.transport_traffic_days_[t]];
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  // Get the routes passing through A
  std::vector<route_idx_t> R;
  std::cout << "Getting the Routes of " << loc_a << std::endl;
  for (auto const r : tt.location_routes_[loc_a]) {
    std::cout << "\t" << r << std::endl;
    R.push_back(r);
  }

  // 1. Run CPU version to collect expected outputs
  std::vector<routing::route_label<64>> expected_outputs;
  for (auto const r : R) {
    auto const seq = tt.route_location_seq_[r];
    for (std::uint16_t s = 0U; s < seq.size() - 1; ++s) {
      stop const s_idx = stop{seq[s]};
      if (s_idx.location_idx() == loc_a) {
        routing::get_earliest_sufficient_transports<64>(
            tt,
            l,
            r,
            s,
            [&](routing::route_label<64> const& out) {
              expected_outputs.push_back(out);
            });
      }
    }
  }

  // 2. Run GPU version
  auto const gpu_outputs = ngpu::get_earliest_sufficient_transports_gpu(tt, gpu_tt, M, 0U, R);

  // 3. Compare sizes and contents
  ASSERT_EQ(expected_outputs.size(), gpu_outputs.size());
  for (size_t i = 0; i < expected_outputs.size(); ++i) {
    EXPECT_EQ(expected_outputs[i].departure_, gpu_outputs[i].departure_);
    EXPECT_EQ(expected_outputs[i].transport_day_offset_, gpu_outputs[i].transport_day_offset_);
    EXPECT_EQ(expected_outputs[i].transport_idx_, gpu_outputs[i].transport_idx_);
    EXPECT_EQ(expected_outputs[i].active_days_, gpu_outputs[i].active_days_);
  }
}

TEST(nigiri_cuda, get_earliest_sufficient_transports_gpu_vs_sequential) {
  auto tt = *timetable::read("timetables/tt-swiss-gouda.bin");
  tt.resolve();

  auto const gpu_tt = ngpu::gpu_timetable{tt};

  std::vector<route_idx_t> R;
  std::deque<cista::bitset<64>> active_days_storage;

  std::string const dump_path = "test/test_data/route-dump.txt";
  std::ifstream f(dump_path);
  ASSERT_TRUE(f.is_open()) << "Failed to open " << dump_path;

  std::string line;
  if (std::getline(f, line)) {
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ';')) {
      if (token.empty()) continue;
      auto const r_val = static_cast<std::uint32_t>(std::stoul(token));
      if (r_val < tt.n_routes()) {
        R.push_back(route_idx_t{r_val});
      }
    }
  }
  std::cout << "Read " << tt.n_locations() << " locations" << std::endl;
  std::cout << "Read " << tt.n_routes() << " locations" << std::endl;
  simple_flat_matrix<std::vector<routing::arrival_label<64>>> M{1U, tt.n_locations()};

  size_t n_labels = 0U;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::size_t pos1 = line.find(';');
    if (pos1 == std::string::npos) continue;
    std::size_t pos2 = line.find(';', pos1 + 1);
    if (pos2 == std::string::npos) continue;
    std::size_t pos3 = line.find(';', pos2 + 1);
    if (pos3 == std::string::npos) continue;
    std::size_t pos4 = line.find(';', pos3 + 1);
    if (pos4 == std::string::npos) continue;

    auto const loc_idx_val = static_cast<std::uint32_t>(std::stoul(line.substr(0, pos1)));
    if (loc_idx_val >= tt.n_locations()) {
      continue;
    }

    auto const departure = static_cast<std::uint16_t>(std::stoul(line.substr(pos1 + 1, pos2 - pos1 - 1)));
    auto const arrival = static_cast<std::uint16_t>(std::stoul(line.substr(pos2 + 1, pos3 - pos2 - 1)));
    auto const arrival_with_transfer = static_cast<std::uint16_t>(std::stoul(line.substr(pos3 + 1, pos4 - pos3 - 1)));
    
    std::string_view bitfield_str = std::string_view(line).substr(pos4 + 1);
    
    routing::arrival_label<64> lbl{
      .arrival_ = arrival,
      .arrival_with_transfer_ = arrival_with_transfer,
      .departure_ = departure,
      .active_days_ = cista::bitset<64>{bitfield_str}
    };

    n_labels++;
    M[0U][loc_idx_val].push_back(lbl);
  }
  std::cout << "Read " << n_labels << " labels" << std::endl;
  std::cout << "Read " << R.size() << " marked routes" << std::endl;

  std::vector<routing::route_label<64>> expected_outputs;
  auto const start_cpu = std::chrono::steady_clock::now();
  for (auto const r : R) {
    auto const seq = tt.route_location_seq_[r];
    for (std::uint16_t s = 0U; s < seq.size() - 1; ++s) {
      stop const s_idx = stop{seq[s]};
      location_idx_t const l = s_idx.location_idx();
      for (auto const& lbl : M[0U][to_idx(l)]) {
        routing::get_earliest_sufficient_transports<64>(
            tt,
            lbl,
            r,
            s,
            [&](routing::route_label<64> const& out) {
              expected_outputs.push_back(out);
            });
      }
    }
  }
  auto const end_cpu = std::chrono::steady_clock::now();
  std::cout << "CPU time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_cpu - start_cpu).count() / 1000.0
            << "ms" << std::endl;

  auto const start_gpu = std::chrono::steady_clock::now();
  auto const gpu_outputs = ngpu::get_earliest_sufficient_transports_gpu(tt, gpu_tt, M, 0U, R);
  auto const end_gpu = std::chrono::steady_clock::now();
  std::cout << "GPU time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_gpu - start_gpu).count() / 1000.0
            << "ms" << std::endl;

  //ASSERT_EQ(expected_outputs.size(), gpu_outputs.size());v
  for (size_t i = 11200; i < 11250; ++i) {
    EXPECT_EQ(expected_outputs[i].departure_, gpu_outputs[i].departure_) << "departure mismatch at index " << i;
    EXPECT_EQ(expected_outputs[i].transport_day_offset_, gpu_outputs[i].transport_day_offset_);
    EXPECT_EQ(expected_outputs[i].transport_idx_, gpu_outputs[i].transport_idx_);
    EXPECT_EQ(expected_outputs[i].active_days_, gpu_outputs[i].active_days_);
  }
}

TEST(nigiri_cuda, get_earliest_sufficient_transports_gpu_vs_sequential_2) {
  auto tt = *timetable::read("timetables/tt-swiss-gouda.bin");
  tt.resolve();

  auto const gpu_tt = ngpu::gpu_timetable{tt};

  std::vector<route_idx_t> R;
  std::deque<cista::bitset<64>> active_days_storage;

  std::string const dump_path = "test/test_data/route-dump.txt";
  std::ifstream f(dump_path);
  ASSERT_TRUE(f.is_open()) << "Failed to open " << dump_path;

  std::string line;
  if (std::getline(f, line)) {
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ';')) {
      if (token.empty()) continue;
      auto const r_val = static_cast<std::uint32_t>(std::stoul(token));
      if (r_val < tt.n_routes() && r_val == 3907) {
        R.emplace_back(r_val);
      }
    }
  }
  std::cout << "Read " << tt.n_locations() << " locations" << std::endl;
  std::cout << "Read " << tt.n_routes() << " locations" << std::endl;
  simple_flat_matrix<std::vector<routing::arrival_label<64>>> M{1U, tt.n_locations()};

  size_t n_labels = 0U;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::size_t pos1 = line.find(';');
    if (pos1 == std::string::npos) continue;
    std::size_t pos2 = line.find(';', pos1 + 1);
    if (pos2 == std::string::npos) continue;
    std::size_t pos3 = line.find(';', pos2 + 1);
    if (pos3 == std::string::npos) continue;
    std::size_t pos4 = line.find(';', pos3 + 1);
    if (pos4 == std::string::npos) continue;

    auto const loc_idx_val = static_cast<std::uint32_t>(std::stoul(line.substr(0, pos1)));
    if (loc_idx_val >= tt.n_locations() || loc_idx_val != 19081) {
      continue;
    }

    auto const departure = static_cast<std::uint16_t>(std::stoul(line.substr(pos1 + 1, pos2 - pos1 - 1)));
    auto const arrival = static_cast<std::uint16_t>(std::stoul(line.substr(pos2 + 1, pos3 - pos2 - 1)));
    auto const arrival_with_transfer = static_cast<std::uint16_t>(std::stoul(line.substr(pos3 + 1, pos4 - pos3 - 1)));

    std::string_view bitfield_str = std::string_view(line).substr(pos4 + 1);

    routing::arrival_label<64> lbl{
      .arrival_ = arrival,
      .arrival_with_transfer_ = arrival_with_transfer,
      .departure_ = departure,
      .active_days_ = cista::bitset<64>{bitfield_str}
    };

    n_labels++;
    M[0U][loc_idx_val].push_back(lbl);
  }
  std::cout << "Read " << n_labels << " labels" << std::endl;
  std::cout << "Read " << R.size() << " marked routes" << std::endl;

  std::vector<routing::route_label<64>> expected_outputs;
  auto const start_cpu = std::chrono::steady_clock::now();
  for (auto const r : R) {
    auto const seq = tt.route_location_seq_[r];
    for (std::uint16_t s = 0U; s < seq.size() - 1; ++s) {
      stop const s_idx = stop{seq[s]};
      location_idx_t const l = s_idx.location_idx();
      for (auto const& lbl : M[0U][to_idx(l)]) {
          routing::get_earliest_sufficient_transports<64>(
              tt, lbl, r, s, [&](routing::route_label<64> const& out) {
                expected_outputs.push_back(out);
              });
      }
    }
  }
  auto const end_cpu = std::chrono::steady_clock::now();
  std::cout << "CPU time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_cpu - start_cpu).count() / 1000.0
            << "ms" << std::endl;

  auto const start_gpu = std::chrono::steady_clock::now();
  auto const gpu_outputs = ngpu::get_earliest_sufficient_transports_gpu(tt, gpu_tt, M, 0U, R);
  auto const end_gpu = std::chrono::steady_clock::now();
  std::cout << "GPU time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_gpu - start_gpu).count() / 1000.0
            << "ms" << std::endl;

  ASSERT_EQ(expected_outputs.size(), gpu_outputs.size());
  for (size_t i = 0U; i < expected_outputs.size(); ++i) {
    EXPECT_EQ(expected_outputs[i].departure_, gpu_outputs[i].departure_) << "departure mismatch at index " << i;
    EXPECT_EQ(expected_outputs[i].transport_day_offset_, gpu_outputs[i].transport_day_offset_) << "transport day offset mismatch at index " << i;
    EXPECT_EQ(expected_outputs[i].transport_idx_, gpu_outputs[i].transport_idx_)  << "transport mismatch at index " << i;
    EXPECT_EQ(expected_outputs[i].active_days_, gpu_outputs[i].active_days_) << "active days mismatch at index " << i;
  }
}

#endif