#include <vector>
#include <string_view>
#include <string>

#include "boost/program_options.hpp"

#include "nigiri/loader/load.h"
#include "date/date.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/para/route_hyper_graph.h"

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

  static constexpr std::array<sub_command, 2> sub_commands = {
    {
      {"export-hgraph", "construct route hgraph from timetable and export it"},
      {"inject-partition", "injects the partitioned hypergraph into a timetable"}
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

    bpo::options_description export_hgraph_desc("export-hgraph options");
    export_hgraph_desc.add_options()
        ("in", bpo::value(&in), "path to the input timetable used to construct the hyper graph")
        ("out", bpo::value(&out)->default_value(out), "path to the output file");

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
    hyper_graph.export_as_hmetis(out);
  } else if (command == "inject-partition") {
    std::cout << "nyi" << std::endl;
  } else {
    std::cout << "Unrecognized command: " << command << std::endl;
  }
}