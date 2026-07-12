#include "gtest/gtest.h"

#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

#include "nigiri/routing/raptor/para/lcl.h"

using namespace nigiri;
using namespace nigiri::routing::para;

TEST(para, lcl_avx512_vs_scalar) {
  std::ifstream f("test/test_data/para/LCL-test1.txt");
  ASSERT_TRUE(f.is_open()) << "Failed to open test data file: test/test_data/para/LCL-test1.txt";

  std::string line;
  // Read route cells (one per line)
  vector_map<route_idx_t, cell_idx_t> route_to_cell_idx;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    auto cell = std::stoul(line);
    route_to_cell_idx.emplace_back(static_cast<std::uint16_t>(cell));
  }

  auto const n_routes = route_to_cell_idx.size();
  ASSERT_GT(n_routes, 0U);

  std::vector<route_partition::global_cell_idx> g_cell_starts = {
    {.cell_idx_ = cell_idx_t{1U}, .level_=0U},
    {.cell_idx_ = cell_idx_t{5U}, .level_=1U},
    {.cell_idx_ = cell_idx_t{12U}, .level_=0U}
  };
  std::vector<route_partition::global_cell_idx> g_cell_dests = {
    {.cell_idx_ = cell_idx_t{3U}, .level_=2U},
    {.cell_idx_ = cell_idx_t{15U}, .level_=0U},
    {.cell_idx_ = cell_idx_t{3U}, .level_=1U}};

  std::vector<rank_t> expected_lcls(n_routes);
  std::vector<rank_t> actual_lcls;
  for (auto batch = 0U; batch < g_cell_starts.size(); ++batch) {
    // Compute reference LCLs using the scalar version
    for (auto r = route_idx_t{0U}; r < n_routes; ++r) {
      expected_lcls[to_idx(r)] =
          std::min(LCL(route_to_cell_idx[r], g_cell_starts[batch]),
                   LCL(route_to_cell_idx[r], g_cell_dests[batch]));
    }

    // Compute LCLs using the AVX-512 version
    compute_min_lcls_avx512(actual_lcls, route_to_cell_idx, g_cell_starts[batch],
                            g_cell_dests[batch], n_routes);

    // Compare results for every route
    ASSERT_EQ(expected_lcls.size(), actual_lcls.size());
    for (size_t r = 0; r < n_routes; ++r) {
      EXPECT_EQ(expected_lcls[r], actual_lcls[r])
          << "Mismatch at route r = " << r;
    }
  }
}
