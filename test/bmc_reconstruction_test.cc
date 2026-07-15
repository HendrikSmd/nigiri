#include "nigiri/routing/raptor/para/bmc_raptor.h"

#include "gtest/gtest.h"

#include <fstream>

#include "cista/containers/bitset.h"

using namespace nigiri;
using namespace nigiri::routing::para;

void new_emplace_relative_journeys_for(
    std::uint32_t const loc_idx,
    simple_flat_matrix<routing::para::bmc_raptor_bag_t> const& M,
    std::vector<bmc_journey>& journeys) {
  std::vector<relative_journey> bag;
  size_t total_size = 0U;
  for (auto k = 1U; k != bmc_raptor::end_k(); ++k) {
    total_size += M[k][loc_idx].size();
  }
  bag.reserve(total_size);

  for (auto k = 1U; k != bmc_raptor::end_k(); ++k) {
    const auto& labels_of_round = M[k][loc_idx];
    for (auto lbl_iter = labels_of_round.begin();
         lbl_iter != labels_of_round.end(); ++lbl_iter) {
      bag.emplace_back(
          lbl_iter->label_.arrival_, lbl_iter->label_.arrival_with_transfer_,
          lbl_iter->label_.departure_, k, lbl_iter, lbl_iter->tdb_);
    }
  }

  std::ranges::sort(bag, std::ranges::less(), &relative_journey::arrival_);
  for (auto i = 0U; i < bag.size() - 1; ++i) {
    auto& focus_label = bag[i];
    if (focus_label.sbf_.none()) {
      continue;
    }

    auto j = i + 1;
    while (j < bag.size()) {
      auto& compare_label = bag[j];
      if (focus_label.arrival_with_transfer_ < compare_label.arrival_) {
        break;
      }
      if (focus_label.arrival_ <= compare_label.arrival_ &&
          focus_label.k_ <= compare_label.k_ &&
          focus_label.departure_ >= compare_label.departure_) {
        compare_label.sbf_ &= ~focus_label.sbf_;
      } else if (focus_label.arrival_ == compare_label.arrival_ &&
                 compare_label.k_ <= focus_label.k_ &&
                 compare_label.departure_ >= focus_label.departure_) {
        focus_label.sbf_ &= ~compare_label.sbf_;
      }
      j++;
    }
  }



  const auto n = bag.size();
  auto right = 0U;

  for (auto left = 0U; left < n; ++left) {
    const auto& left_lbl = bag[left];
    // Move the right pointer until the difference condition is met
    while (right < n && bag[right].arrival_ - left_lbl.arrival_ < 1440) {
      right++;
    }

    // All elements from 'right' to 'n-1' are valid pairs with 'left'
    for (auto i = right; i < n; ++i) {
      auto& right_lbl = bag[i];
      auto shift = 1U;
      while (left_lbl.arrival_ + shift * 1440 <= right_lbl.arrival_) {
        auto new_arrival = left_lbl.arrival_ + shift * 1440;
        if (new_arrival <= right_lbl.arrival_ && left_lbl.k_ <= right_lbl.k_) {
          right_lbl.sbf_ &= ~(left_lbl.sbf_ >> shift);
        } else {
          break;
        }
        shift++;
      }
    }
  }
  for (const auto& lbl : bag) {
    auto const& tdb = lbl.sbf_;
    tdb.for_each_set_bit([&](size_t const i) {
      journeys.emplace_back(routing::para::routing_time{static_cast<int>(
           i * 1440 + lbl.arrival_)},
       routing::para::routing_time{static_cast<int>(
           i * 1440 + lbl.departure_)},
       static_cast<std::uint16_t>(lbl.k_ > 0 ? lbl.k_ - 1 : 0U),
       M[0][0].begin());
    });
     }
}

void emplace_relative_journeys_for(
    std::uint32_t const loc_idx,
    simple_flat_matrix<routing::para::bmc_raptor_bag_t> const& M,
    std::vector<bmc_journey>& bag) {
  constexpr auto dom = [](bmc_journey const& l1,
                          bmc_journey const& l2) {
    return bmc_journey::dominates(l1, l2);
  };

  for (auto k = 1U; k != bmc_raptor::end_k(); ++k) {
    auto const& round_bag = M[k][loc_idx];
    if (round_bag.size() == 0) {
      continue;
    }

    for (auto label_it = round_bag.begin(); label_it != round_bag.end();
         ++label_it) {
      auto const label_view = *label_it;
      auto const& tdb = label_view.tdb_;
      tdb.for_each_set_bit([&](size_t const i) {
        pareto_utils<bmc_journey>::pareto_add(
            bag,
            {.arrival_ = routing::para::routing_time{static_cast<int>(
                 i * 1440 + label_view.label_.arrival_)},
             .departure_ = routing::para::routing_time{static_cast<int>(
                 i * 1440 + label_view.label_.departure_)},
             .transfers_ = static_cast<std::uint16_t>(k > 0 ? k - 1 : 0U),
             .label_iter_ = label_it},
            dom);
      });
    }
  }
}

TEST(nigiri_bmc, reconstruction_test) {
  const auto n_locations = 801;
  std::string const dump_path = "test/test_data/journey-dump1.txt";
  std::ifstream f(dump_path);
  ASSERT_TRUE(f.is_open()) << "Failed to open " << dump_path;


  simple_flat_matrix<routing::para::bmc_raptor_bag_t> M{bmc_raptor::end_k(), n_locations};

  size_t n_labels = 0U;
  std::string line;
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
    std::size_t pos5 = line.find(';', pos4 + 1);
    if (pos5 == std::string::npos) continue;

    auto const round_k = static_cast<std::uint32_t>(std::stoul(line.substr(0, pos1)));
    if (round_k >= bmc_raptor::end_k()) {
      continue;
    }

    auto const loc_idx_val = static_cast<std::uint16_t>(std::stoul(line.substr(pos1 + 1, pos2 - pos1 - 1)));
    if (loc_idx_val >= n_locations) {
      continue;
    }

    auto const departure = static_cast<std::uint16_t>(std::stoul(line.substr(pos2 + 1, pos3 - pos2 - 1)));
    auto const arrival = static_cast<std::uint16_t>(std::stoul(line.substr(pos3 + 1, pos4 - pos3 - 1)));
    auto const arrival_with_transfers = static_cast<std::uint16_t>(std::stoul(line.substr(pos4 + 1, pos5 - pos4 - 1)));
    std::string_view bitfield_str = std::string_view(line).substr(pos5 + 1);
    routing::para::bmc_raptor_label lbl{
      .route_idx_ = 0U,
      .enter_stop_idx_ = 0U,
      .exit_stop_idx_ = 0U,
      .arrival_ = arrival,
      .parent_bag_idx_ = 0U,
      .arrival_with_transfer_ = arrival_with_transfers,
      .departure_ = departure,
      .is_footpath_ = 0,
      .has_parent_ = 0,
      .reserved_ = 0
    };

    n_labels++;
    M[round_k][loc_idx_val].labels_.emplace_back(lbl, search_bitfield(bitfield_str));
  }
  std::cout << "Read " << n_labels << " labels" << std::endl;

  EXPECT_EQ(n_labels, 472003);
  std::vector old_journeys(n_locations, std::vector<bmc_journey>{});
  {
    auto timer = scoped_timer("Old Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      emplace_relative_journeys_for(i, M, old_journeys[i]);
    }
  }
  std::vector new_journeys(n_locations, std::vector<bmc_journey>{});
  {
    auto timer = scoped_timer("New Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      new_emplace_relative_journeys_for(i, M, new_journeys[i]);
    }
  }
  for (auto i = 0U; i < n_locations; ++i) {
    EXPECT_EQ(old_journeys[i].size(), new_journeys[i].size()) << "location " << i;
    EXPECT_TRUE(std::ranges::is_permutation(
    old_journeys[i], new_journeys[i], [](auto const& lhs, auto const& rhs) {
      return lhs.arrival_ == rhs.arrival_ &&
             lhs.departure_ == rhs.departure_ &&
             lhs.transfers_ == rhs.transfers_;
    }));
  }
}

constexpr auto const journeys_20 = R"(
1;0;1225;1240;1242;0000000000000000000000000000000000000000000000001000000000000000
1;0;1245;2680;2682;0000000000000000000000000000000000000000000000000100000000000000
1;0;1245;4120;4122;0000000000000000000000000000000000000000000000000010000000000000
1;0;1365;8440;8442;0000000000000000000000000000000000000000000000000000010000000000
1;0;1345;1360;1362;0000000000000000000000000000000000000000000000001000000000000000
1;0;1285;1300;1302;0000000000000000000000000000000000000000000000001000000000000000
2;0;1426;2680;2682;0000000000000000000000000000000000000000000000000100000000000000
2;0;1394;4120;4122;0000000000000000000000000000000000000000000000000010000000000000
2;0;1394;5560;5562;0000000000000000000000000000000000000000000000000001000000000000
2;0;1394;7000;7002;0000000000000000000000000000000000000000000000000000100000000000
2;0;1394;8440;8442;0000000000000000000000000000000000000000000000000000010000000000
2;0;1154;1240;1242;0000000000000000000000000000000000000000000000001000000000000000
3;0;1432;5560;5562;0000000000000000000000000000000000000000000000000001000000000000
3;0;1432;7000;7002;0000000000000000000000000000000000000000000000000000100000000000
3;0;1432;8440;8442;0000000000000000000000000000000000000000000000000000010000000000
3;0;1430;2680;2682;0000000000000000000000000000000000000000000000000100000000000000
3;0;1432;4120;4122;0000000000000000000000000000000000000000000000000010000000000000
3;0;1209;1240;1242;0000000000000000000000000000000000000000000000001000000000000000
3;0;1328;1360;1362;0000000000000000000000000000000000000000000000001000000000000000
3;0;1269;1300;1302;0000000000000000000000000000000000000000000000001000000000000000
3;0;1426;2655;2655;0000000000000000000000000000000000000000000000000100000000000000
3;0;1394;4095;4095;0000000000000000000000000000000000000000000000000010000000000000
3;0;1394;5535;5535;0000000000000000000000000000000000000000000000000001000000000000
3;0;1394;6975;6975;0000000000000000000000000000000000000000000000000000100000000000
3;0;1394;8415;8415;0000000000000000000000000000000000000000000000000000010000000000
3;0;374;1215;1215;0000000000000000000000000000000000000000000000001000000000000000
4;0;1171;1240;1242;0000000000000000000000000000000000000000000000001000000000000000
4;0;1430;2655;2655;0000000000000000000000000000000000000000000000000100000000000000
4;0;1432;5535;5535;0000000000000000000000000000000000000000000000000001000000000000
4;0;1432;6975;6975;0000000000000000000000000000000000000000000000000000100000000000
4;0;1432;8415;8415;0000000000000000000000000000000000000000000000000000010000000000
4;0;1432;4095;4095;0000000000000000000000000000000000000000000000000010000000000000
4;0;1092;1215;1215;0000000000000000000000000000000000000000000000001000000000000000
5;0;1112;1215;1215;0000000000000000000000000000000000000000000000001000000000000000
)";

TEST(nigiri_bmc, reconstruction_test2) {
  const auto n_locations = 1;
  std::stringstream ss(journeys_20);
  simple_flat_matrix<routing::para::bmc_raptor_bag_t> M{routing::para::bmc_raptor::end_k(), n_locations};

  size_t n_labels = 0U;
  std::string line;
  while (std::getline(ss, line)) {
    if (line.empty()) continue;
    std::size_t pos1 = line.find(';');
    if (pos1 == std::string::npos) continue;
    std::size_t pos2 = line.find(';', pos1 + 1);
    if (pos2 == std::string::npos) continue;
    std::size_t pos3 = line.find(';', pos2 + 1);
    if (pos3 == std::string::npos) continue;
    std::size_t pos4 = line.find(';', pos3 + 1);
    if (pos4 == std::string::npos) continue;
    std::size_t pos5 = line.find(';', pos4 + 1);
    if (pos5 == std::string::npos) continue;

    auto const round_k = static_cast<std::uint32_t>(std::stoul(line.substr(0, pos1)));
    if (round_k >= routing::para::bmc_raptor::end_k()) {
      continue;
    }

    auto const loc_idx_val = static_cast<std::uint16_t>(std::stoul(line.substr(pos1 + 1, pos2 - pos1 - 1)));
    if (loc_idx_val >= n_locations) {
      continue;
    }

    auto const departure = static_cast<std::uint16_t>(std::stoul(line.substr(pos2 + 1, pos3 - pos2 - 1)));
    auto const arrival = static_cast<std::uint16_t>(std::stoul(line.substr(pos3 + 1, pos4 - pos3 - 1)));
    auto const arrival_with_transfers = static_cast<std::uint16_t>(std::stoul(line.substr(pos4 + 1, pos5 - pos4 - 1)));
    std::string_view bitfield_str = std::string_view(line).substr(pos5 + 1);
    routing::para::bmc_raptor_label lbl{
      .route_idx_ = 0U,
      .enter_stop_idx_ = 0U,
      .exit_stop_idx_ = 0U,
      .arrival_ = arrival,
      .parent_bag_idx_ = 0U,
      .arrival_with_transfer_ = arrival_with_transfers,
      .departure_ = departure,
      .is_footpath_ = 0,
      .has_parent_ = 0,
      .reserved_ = 0
    };

    n_labels++;
    M[round_k][loc_idx_val].labels_.emplace_back(lbl, search_bitfield(bitfield_str));
  }
  std::cout << "Read " << n_labels << " labels" << std::endl;

  EXPECT_EQ(n_labels, 34);
  std::vector old_journeys(n_locations, std::vector<routing::para::bmc_journey>{});
  {
    auto timer = scoped_timer("Old Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      emplace_relative_journeys_for(i, M, old_journeys[i]);
    }
  }
  std::vector new_journeys(n_locations, std::vector<routing::para::bmc_journey>{});
  {
    auto timer = scoped_timer("New Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      new_emplace_relative_journeys_for(i, M, new_journeys[i]);
    }
  }
  for (auto i = 0U; i < n_locations; ++i) {
    EXPECT_EQ(old_journeys[i].size(), new_journeys[i].size()) << "location " << i;
    EXPECT_TRUE(std::ranges::is_permutation(old_journeys[i], new_journeys[i], [](const auto& lhs, const auto& rhs) {
      return lhs.arrival_ == rhs.arrival_ && lhs.departure_ == rhs.departure_ && lhs.transfers_ == rhs.transfers_;
    }));
  }
}

TEST(nigiri_bmc, reconstruction_test3) {
  auto const n_locations = 1;
  std::string const dump_path = "test/test_data/journey-dump1.txt";
  std::ifstream f(dump_path);
  ASSERT_TRUE(f.is_open()) << "Failed to open " << dump_path;

  simple_flat_matrix<routing::para::bmc_raptor_bag_t> M{
      routing::para::bmc_raptor::end_k(), n_locations};

  size_t n_labels = 0U;
  std::string line;
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
    std::size_t pos5 = line.find(';', pos4 + 1);
    if (pos5 == std::string::npos) continue;

    auto const round_k =
        static_cast<std::uint32_t>(std::stoul(line.substr(0, pos1)));
    if (round_k >= routing::para::bmc_raptor::end_k()) {
      continue;
    }

    auto const loc_idx_val = static_cast<std::uint16_t>(
        std::stoul(line.substr(pos1 + 1, pos2 - pos1 - 1)));
    if (loc_idx_val != 179) {
      continue;
    }

    auto const departure = static_cast<std::uint16_t>(
        std::stoul(line.substr(pos2 + 1, pos3 - pos2 - 1)));
    auto const arrival = static_cast<std::uint16_t>(
        std::stoul(line.substr(pos3 + 1, pos4 - pos3 - 1)));
    auto const arrival_with_transfers = static_cast<std::uint16_t>(
        std::stoul(line.substr(pos4 + 1, pos5 - pos4 - 1)));
    std::string_view bitfield_str = std::string_view(line).substr(pos5 + 1);
    routing::para::bmc_raptor_label lbl{
        .route_idx_ = 0U,
        .enter_stop_idx_ = 0U,
        .exit_stop_idx_ = 0U,
        .arrival_ = arrival,
        .parent_bag_idx_ = 0U,
        .arrival_with_transfer_ = arrival_with_transfers,
        .departure_ = departure,
        .is_footpath_ = 0,
        .has_parent_ = 0,
        .reserved_ = 0};

    n_labels++;
    M[round_k][0].labels_.emplace_back(lbl, search_bitfield(bitfield_str));
  }
  std::cout << "Read " << n_labels << " labels" << std::endl;

  EXPECT_EQ(n_labels, 580);
  std::vector old_journeys(n_locations,
                           std::vector<routing::para::bmc_journey>{});
  {
    auto timer = scoped_timer("Old Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      emplace_relative_journeys_for(i, M, old_journeys[i]);
    }
  }
  std::vector new_journeys(n_locations,
                           std::vector<routing::para::bmc_journey>{});
  {
    auto timer = scoped_timer("New Emplace");
    for (auto i = 0U; i < n_locations; ++i) {
      new_emplace_relative_journeys_for(i, M, new_journeys[i]);
    }
  }
  for (auto i = 0U; i < n_locations; ++i) {
    EXPECT_EQ(old_journeys[i].size(), new_journeys[i].size())
        << "location " << i;
    EXPECT_TRUE(std::ranges::is_permutation(
        old_journeys[i], new_journeys[i], [](auto const& lhs, auto const& rhs) {
          return lhs.arrival_ == rhs.arrival_ &&
                 lhs.departure_ == rhs.departure_ &&
                 lhs.transfers_ == rhs.transfers_;
        }));
  }
}