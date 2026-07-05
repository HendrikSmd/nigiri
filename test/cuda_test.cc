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
  auto M = simple_flat_matrix<std::vector<routing::route_label<64>>>{1U, n_locations};

  // Find location "A" ("0000001")
  auto const loc_a = tt.locations_.location_id_to_idx_.at({"0000001", src});

  // Create an active days bitset (day 0 active)
  cista::bitset<64> active_days;
  active_days.set(0, true);

  // Add a label at location A
  // departure = 05:00 (300 minutes after midnight)
  // arrival = 07:00 (420 minutes after midnight)
  // arrival_with_transfer = 07:00
  routing::route_label<64> l{
      .arrival_ = 420,
      .arrival_with_transfer_ = 420,
      .departure_ = 300,
      .active_days_ = active_days
  };
  M[0][to_idx(loc_a)].push_back(l);

  // Get the routes passing through A
  std::vector<route_idx_t> R;
  for (auto const r : tt.location_routes_[loc_a]) {
    R.push_back(r);
  }

  // 1. Run CPU version to collect expected outputs
  std::vector<routing::route_label_by_value<64>> expected_outputs;
  for (auto const r : R) {
    auto const seq = tt.route_location_seq_[r];
    for (std::uint16_t s = 0U; s < seq.size(); ++s) {
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
    EXPECT_EQ(expected_outputs[i].arrival_, gpu_outputs[i].arrival_);
    EXPECT_EQ(expected_outputs[i].arrival_with_transfer_, gpu_outputs[i].arrival_with_transfer_);
    EXPECT_EQ(expected_outputs[i].departure_, gpu_outputs[i].departure_);
    EXPECT_EQ(expected_outputs[i].active_days_, gpu_outputs[i].active_days_);
  }
}

#endif