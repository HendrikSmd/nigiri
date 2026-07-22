#pragma once

#include <iosfwd>

#include "nigiri/timetable.h"

namespace nigiri {

enum class timetable_visualization_format { kSvg, kTikz };

struct timetable_visualization_options {
  timetable_visualization_format format_ = timetable_visualization_format::kSvg;
  double width_ = 1000.0;
  double point_radius_ = 0.03;
  double padding_ = 0.02;
  bool background_ = true;
};

void visualize_timetable(timetable const&, std::ostream&,
                         timetable_visualization_options const& = {});

}  // namespace nigiri
