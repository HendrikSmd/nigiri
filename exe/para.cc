#include <vector>
#include <string_view>
#include <string>

#include "boost/program_options.hpp"

#include "nigiri/loader/load.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/raptor/para/export_partition.h"
#include "nigiri/routing/raptor/para/route_hyper_graph.h"
#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/raptor/raptor_state.h"
#include "nigiri/routing/search.h"
#include "nigiri/timetable.h"

#include "date/date.h"

namespace fs = std::filesystem;
namespace bpo = boost::program_options;
using namespace nigiri;

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

int main(int argc, char** argv) {

  static constexpr std::array<sub_command, 3> sub_commands = {
    {
      {"export-hgraph", "construct route hgraph from timetable and export it"},
      {"import-partition", "imports a partition file"},
      {"export-partition", "exports a partition in a supported format"}
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

    bpo::options_description export_hgraph_desc("export-hgraph options");
    export_hgraph_desc.add_options()
        ("in", bpo::value(&in), "path to the input timetable used to construct the hyper graph")
        ("out", bpo::value(&out)->default_value(out), "path to the output file")
        ("node_weights", bpo::value(&node_weights)->default_value(node_weights), "compute node weights")
        ("hedge_weights", bpo::value(&hedge_weights)->default_value(hedge_weights), "compute hedge weights");

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

    routing::route_hyper_graph hyper_graph;
    hyper_graph.from(tt);
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

    routing::route_partition partition;
    partition.from_hmetis_result(in_part, tt);
    fmt::print("Finished reading in route partition with {} levels\n", partition.n_levels_);

    const auto count_cells = [](routing::route_partition const& partition, std::vector<size_t>& counts) {
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

    auto route_part = *routing::route_partition::read(in_part);

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
  } else if (command == "customize") {
    auto tt = *timetable::read("../cmake-build-debug-clang-17/tt-swiss.bin");
    tt.resolve();
    std::cout << tt.internal_interval() << std::endl;
    location_idx_t dest{29773};
    location_idx_t src{17093};

    auto q = routing::query{
      .start_time_ = interval{unixtime_t{sys_days{2024_y / March / 10}} + 8_hours, unixtime_t{sys_days{2024_y / March / 10}} + 10_hours},
      .use_start_footpaths_ = true,
      .start_ = {{src, 0_minutes, 0U}},
      .destination_ = {{dest, 0_minutes, 0U}},
      .prf_idx_ = 0,
    };
    const auto res = raptor_search(tt, q);
    std::cout << res.size() << std::endl;
    for (const auto& j : res) {
      std::cout << j.departure_time() << ", " << j.arrival_time() << "@" << j.dest_ << std::endl;
    }

    //for (auto l = location_idx_t{0}; l < tt.n_locations(); ++l) {
    //  const auto& name = tt.locations_.names_[l];
   //   if (name.view().contains("Zürich")) {
     //   std::cout << l << ": " << name.view() << std::endl;
     // }
    } else {
    std::cout << "Unrecognized command: " << command << std::endl;
  }
}