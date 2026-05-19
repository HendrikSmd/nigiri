#include <vector>
#include <string_view>
#include <sstream>
#include <chrono>
#include <string>

#include "boost/program_options.hpp"

#include "nigiri/loader/load.h"
#include "nigiri/common/clique.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/raptor/para/bmc_raptor.h"
#include "nigiri/routing/raptor/para/customization.h"
#include "nigiri/routing/raptor/para/export_partition.h"
#include "nigiri/routing/raptor/para/mc_raptor_search.h"
#include "nigiri/routing/raptor/para/route_hyper_graph.h"
#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/raptor/raptor_state.h"
#include "nigiri/routing/raptor_search.h"
#include "nigiri/routing/search.h"
#include "nigiri/timetable.h"

#include "date/date.h"

namespace fs = std::filesystem;
namespace bpo = boost::program_options;
using namespace nigiri;
using namespace date;

struct sub_command {
  std::string_view literal;
  std::string_view description;
};

template<std::size_t N>
constexpr size_t get_max_sub_command_length(const std::array<sub_command, N>& arr) {
  return std::accumulate(
    arr.begin(),
    arr.end(),
    size_t{0},
    [](size_t current_max, const sub_command& sc) {
      return std::max(current_max, sc.literal.length());
    }
  );
}

pareto_set<routing::journey> raptor_search(
    timetable const& tt, routing::query q) {
  using namespace nigiri;
  using algo_state_t = routing::raptor_state;
  static auto search_state = routing::search_state{};
  static auto algo_state = algo_state_t{};

  return *(routing::raptor_search(tt, nullptr, search_state, algo_state,
                                  std::move(q), direction::kForward)
               .journeys_);
}

pareto_set<routing::journey> para_raptor_search(
    timetable const& tt, routing::para::route_rank_store const& rs, routing::query q) {
  using namespace nigiri;
  using algo_state_t = routing::raptor_state;
  static auto search_state = routing::search_state{};
  static auto algo_state = algo_state_t{};

  return *(routing::para_raptor_search(tt, rs, search_state, algo_state,
                                       std::move(q), std::nullopt)
               .journeys_);
}

std::vector<routing::para::bmc_journey> bmc_raptor_search(
    timetable const& tt, routing::query q) {
  using namespace nigiri;
  routing::para::bmc_raptor_state state;
  bitvec route_mask = bitvec::max(tt.n_routes());
  bitvec transfer_mask = bitvec::max(tt.n_locations());
  bitvec destination_mask(tt.n_locations());
  routing::para::timetable_view tt_view(tt, route_mask);

  routing::para::bmc_raptor raptor(
    tt_view, state, destination_mask, transfer_mask);

  const auto start_loc = q.start_.front().target();
  const auto dest_loc = q.destination_.front().target();

  destination_mask.set(to_idx(dest_loc), true);
  raptor.init_starts(tt_view.get_view_idx(start_loc), true);

  raptor.rounds();

  std::vector<routing::para::bmc_journey> result;
  raptor.emplace_relative_journeys_for(tt_view.get_view_idx(dest_loc), result);
  return result;
}

int main(int argc, char** argv) {

  static constexpr std::array<sub_command, 7> sub_commands = {
    {
      {"export-hgraph", "construct route hgraph from timetable and export it"},
      {"import-partition", "imports a partition file"},
      {"export-partition", "exports a partition in a supported format"},
      {"start-customization", "start the customization process"},
      {"inspect-rank-store", "outputs information on the given rank store"},
      {"clique-cover", "computing a clique cover for the foot-graph of the given timetable"},
      {"check-fp-transitivity", "checks if the given footpaths in a timetable are transitively closes"}
    }
  };

  constexpr size_t max_sub_command_length = get_max_sub_command_length(sub_commands);


  bpo::options_description global_desc("Global options");
  global_desc.add_options()
      ("help,h", "produce this help message")
      ("debug", "Turn on debug output")
      ("command", bpo::value<std::string>(), "command to execute")
      ("subargs", bpo::value<std::vector<std::string> >(), "Arguments for command");

  bpo::positional_options_description pos;
  pos.add("command", 1).
      add("subargs", -1);

  bpo::variables_map vm;

  bpo::parsed_options parsed = bpo::command_line_parser(argc, argv).
      options(global_desc).
      positional(pos).
      allow_unregistered().
      run();

  bpo::store(parsed, vm);

  if (!vm.contains("command")) {
    if (vm.contains("help")) {
      std::cout << global_desc << "\n\n";
      std::cout << "Possible commands: " << std::endl;
      for (const auto& command : sub_commands) {
        std::cout << "  " << std::left << std::setw(max_sub_command_length) << command.literal << "\t\t" << command.description << std::endl;
      }
      return 0;
    }
    std::cout << "You have to specify a command, for a list of possible commands run with flag --help" << std::endl;
    return 0;
  }

  std::string command = vm["command"].as<std::string>();

  bpo::variables_map cvm;
  if(command == "export-hgraph") {
    auto in = fs::path{};
    auto out = fs::path{"hyper-graph.out.txt"};
    auto node_weights = true;
    auto hedge_weights = true;
    std::string hedge_weighting_scheme = "numEvents";
    std::string hedge_normalization = "none";

    bpo::options_description export_hgraph_desc("export-hgraph options");
    export_hgraph_desc.add_options()(
        "in", bpo::value(&in),
        "path to the input timetable used to construct the hyper graph")(
        "out", bpo::value(&out)->default_value(out), "path to the output file")(
        "node_weights", bpo::value(&node_weights)->default_value(node_weights),
        "compute node weights")(
        "hedge_weights",
        bpo::value(&hedge_weights)->default_value(hedge_weights),
        "compute hedge weights")(
        "hedge_weighting",
        bpo::value(&hedge_weighting_scheme)
            ->default_value(hedge_weighting_scheme),
        "hedge weighting scheme (numRoutes)=number of routes incident to "
        "component, (numEvents)=num events incident to component")(
        "hedge_normalization",
        bpo::value(&hedge_normalization)->default_value(hedge_normalization),
        "hedge normalization (none)=raw weights, (log)=log normalization, "
        "(cmpntSize)=normalized by component size, "
        "(logAndCmpntSize)=normalized by component size and then log "
        "normalized");

    if (vm.contains("help")) {
      std::cout << export_hgraph_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(export_hgraph_desc).run(), cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in);
    tt.resolve();

    auto hedge_weighting = routing::hedge_weighting::kNumEvents;
    if (hedge_weighting_scheme == "numEvents") {
      hedge_weighting = routing::hedge_weighting::kNumEvents;
    } else if (hedge_weighting_scheme == "numRoutes") {
      hedge_weighting = routing::hedge_weighting::kNumRoutes;
    } else {
      utl::fail("Invalid hedge weighting scheme");
    }

    auto normalization = routing::hedge_normalization::kNone;
    if (hedge_normalization == "none") {
      normalization = routing::hedge_normalization::kNone;
    } else if (hedge_normalization == "log") {
      normalization = routing::hedge_normalization::kLog;
    } else if (hedge_normalization == "cmpntSize") {
      normalization = routing::hedge_normalization::kCmpntSize;
    } else if (hedge_normalization == "logAndCmpntSize") {
      normalization = routing::hedge_normalization::kLogCmpntSize;
    } else {
      utl::fail("Invalid hedge normalization");
    }
    nigiri::log(log_lvl::info, "hyper-graph export",
                "Hedge weighting scheme: {}",
                (hedge_weighting == routing::hedge_weighting::kNumRoutes)
                    ? "numRoutes"
                    : "numEvents");

    nigiri::log(
        log_lvl::info, "hyper-graph export", "Hedge normalization: {}",
        (normalization == routing::hedge_normalization::kNone)
            ? "none"
            : (normalization == routing::hedge_normalization::kLog
                   ? "log"
                   : (normalization == routing::hedge_normalization::kCmpntSize
                          ? "cmpntSize"
                          : "logAndCmpntSize")));
    routing::route_hyper_graph hyper_graph;
    hyper_graph.from(tt, hedge_weighting, normalization);
    hyper_graph.export_as_hmetis(out, node_weights, hedge_weights);
  } else if (command == "import-partition") {
    auto in_part = fs::path{};
    auto in_tt = fs::path{};
    auto out = fs::path{"part.bin"};

    bpo::options_description import_part_desc("export-hgraph options");
    import_part_desc.add_options()
        ("in_part", bpo::value(&in_part), "path to the partition file in hmetis format")
        ("in_tt", bpo::value(&in_tt), "path to the timetable")
        ("out", bpo::value(&out)->default_value(out), "path to the output file");

    if (vm.contains("help")) {
      std::cout << import_part_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(import_part_desc).run(), cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    routing::para::route_partition partition;
    partition.from_hmetis_result(in_part, tt);
    fmt::print("Finished reading in route partition with {} levels\n", partition.n_levels_);

    const auto count_cells = [](routing::para::route_partition const& partition, std::vector<size_t>& counts) {
      auto const n_cells = 1U << partition.n_levels_;
      counts.resize(n_cells, 0U);
      for (const auto& cell_idx : partition.route_to_cell_idx_) {
        counts[cell_idx.v_]++;
      }
    };

    std::vector<size_t> counts;
    count_cells(partition, counts);
    std::cout << "Count per cell: " << std::endl;
    for (size_t cell_idx = 0U; cell_idx < counts.size(); ++cell_idx) {
      std::cout << "  Cell [" << std::right << std::setw(10) << cell_idx << "]: #" << counts[cell_idx] << std::endl;
    }

    partition.write(out);
  } else if (command == "export-partition") {
    auto in_part = fs::path{};
    auto in_tt = fs::path{};
    auto out = fs::path{};
    auto format = size_t{0};

    bpo::options_description export_part_desc("export-hgraph options");
    export_part_desc.add_options()
        ("in_part", bpo::value(&in_part), "path to the route partition file (nigiri format)")
        ("in_tt", bpo::value(&in_tt), "path to the timetable")
        ("format", bpo::value(&format)->default_value(format), "format: (0)json, (1)graphviz, ...")
        ("out", bpo::value(&out), "path to the output file");


    if (vm.contains("help")) {
      std::cout << export_part_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(export_part_desc).run(), cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    auto route_part = *routing::para::route_partition::read(in_part);

    static constexpr std::array<std::string_view, 2> format_to_suffix = {
      ".json",
      ".dot"
    };

    auto out_str = out.string();
    const auto dot_pos = out_str.find_last_of(".");
    if (dot_pos != std::string::npos) {
      const auto suffix = out_str.substr(dot_pos, std::string::npos);
      if (suffix != format_to_suffix[format]) {
        out_str += format_to_suffix[format];
      }
    }

    std::ofstream out_file(out_str, std::ios::out);
    switch (format) {
      case 0:
        out_file << routing::para::to_featurecollection(tt, route_part);
        break;
      case 1:
        out_file << routing::para::to_graphviz(tt, route_part);
        break;
      default:
        std::cout << "invalid format!" << std::endl;
    }
  } else if (command == "start-customization") {
    auto in_part = fs::path{};
    auto in_tt = fs::path{};
    auto out = fs::path{"store.bin"};

    bpo::options_description start_custom_desc("export-hgraph options");
    start_custom_desc.add_options()
        ("in_part", bpo::value(&in_part), "path to the route partition file (nigiri format)")
        ("in_tt", bpo::value(&in_tt), "path to the timetable")
        ("out", bpo::value(&out)->default_value(out), "path to the output file");


    if (vm.contains("help")) {
      std::cout << start_custom_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(start_custom_desc).run(), cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    routing::para::customizer customizer{tt};
    auto const store = customizer.construct_route_rank_store(
      std::move(*routing::para::route_partition::read(in_part)));
    store.print_summary(std::cout);
    store.write(out);

  } else if (command == "inspect-rank-store") {
    auto in_store = fs::path{};

    bpo::options_description inspect_rank_store("inspect-rank-store options");
    inspect_rank_store.add_options()
        ("in_store", bpo::value(&in_store), "path to the rank store");

    if (vm.contains("help")) {
      std::cout << inspect_rank_store << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(inspect_rank_store).run(), cvm);
    bpo::notify(cvm);
    auto store = *routing::para::route_rank_store::read(in_store);
    store.print_summary(std::cout);
  } else if (command == "clique-cover") {
    auto in_tt = fs::path{};

    bpo::options_description start_custom_desc("clique-cover options");
    start_custom_desc.add_options()
        ("in_tt", bpo::value(&in_tt), "path to the timetable");


    if (vm.contains("help")) {
      std::cout << start_custom_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(start_custom_desc).run(), cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    {
      auto const timer = scoped_timer("Computing clique cover");
      auto const cliques = clique_cover(tt);
      std::cout << "Number of cliques: " << cliques.size()
                << " vs. Number of components: "
                << tt.component_locations_.size() << " vs. Number of locations "
                << tt.n_locations() << std::endl;
    }
  } else if (command == "check-fp-transitivity") {
    auto in_tt = fs::path{};

    bpo::options_description start_custom_desc("check-fp-transitivity options");
    start_custom_desc.add_options()("in_tt", bpo::value(&in_tt),
                                    "path to the timetable with the footpaths");

    if (vm.contains("help")) {
      std::cout << start_custom_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts =
        bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(start_custom_desc).run(),
               cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    size_t count = 0U;
    size_t non_transitive = 0U;
    for (auto loc = location_idx_t{0U}; loc < tt.n_locations(); ++loc) {
      const auto& first_fps = tt.locations_.footpaths_out_[kDefaultProfile][loc];
      for (const auto& first_fp : first_fps) {
        const auto middle_loc = first_fp.target();
        const auto& second_fps = tt.locations_.footpaths_out_[kDefaultProfile][middle_loc];
        for (const auto& second_fp : second_fps) {
          const auto target_loc = second_fp.target();

          if (loc == target_loc) {
            continue;
          }

          auto const closure = std::find_if(first_fps.begin(), first_fps.end(),
                                            [target_loc](auto const& fp) {
                                              return fp.target() == target_loc;
                                            });
          if (closure == first_fps.end()) {
            non_transitive++;
          }
          count++;
        }
      }
    }

    if (non_transitive == 0U) {
      std::cout << "Footpaths are transitively closed!" << std::endl;
    } else {
      std::cout << "Found " << non_transitive << "/" << count << " many occurrences of non transitivity!" << std::endl;
    }
  } else if (command == "route") {
    auto in_tt = fs::path{};
    auto in_rs = fs::path{};

    bpo::options_description start_custom_desc("route options");
    start_custom_desc.add_options()
      ("in_tt", bpo::value(&in_tt), "path to the timetable")
      ("in_rs", bpo::value(&in_rs), "path to the route rank store");

    if (vm.contains("help")) {
      std::cout << start_custom_desc << "\n\n";
      return 0;
    }

    std::vector<std::string> opts =
        bpo::collect_unrecognized(parsed.options, bpo::include_positional);
    opts.erase(opts.begin());

    bpo::store(bpo::command_line_parser(opts).options(start_custom_desc).run(),
               cvm);
    bpo::notify(cvm);

    auto tt = *timetable::read(in_tt);
    tt.resolve();

    auto rs = *routing::para::route_rank_store::read(in_rs);

    interval<unixtime_t> start_time = {
      .from_ = sys_days(2026_y / March / 16_d) + 15_hours +  56_minutes,
      .to_ = sys_days(2026_y / March / 16_d) + 17_hours + 56_minutes,
    };

    const auto start_loc = location_idx_t{123215};
    const auto dest_loc = location_idx_t{93506};

    routing::query q = {
      .start_time_ = start_time,
      .use_start_footpaths_ = true,
      .start_ = {{start_loc, 0_minutes, 0U}},
      .destination_ = {{dest_loc, 0_minutes, 0U}}
    };

    auto res = raptor_search(tt, q);
    std::cout << res.size() << std::endl;
    for (auto& j : res) {
      j.print(std::cout, tt);
    }
    std::cout << "bmc raptor:" << std::endl;
    auto res_bmc = bmc_raptor_search(tt, q);
    for (const auto j : res_bmc) {
      std::cout << "dep: " << j.departure_.to_unixtime(tt) << ", arr: " << j.arrival_.to_unixtime(tt) << " transfers: " << j.transfers_ << std::endl;
    }
  } else {
    std::cout << "Unrecognized command: " << command << std::endl;
  }
}