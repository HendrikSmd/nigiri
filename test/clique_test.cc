#include "gtest/gtest.h"

#include "nigiri/common/clique.h"

using namespace nigiri;

TEST(clique, bridge) {
  adjacency_matrix const matrix = {
    bitvec{"10"},
    bitvec{"01"},
  };

  const auto cliques = clique_cover(matrix);
  EXPECT_EQ(cliques.size(), 1);
  EXPECT_EQ(cliques[0].count(), 2);
  EXPECT_EQ(cliques[0], bitvec::max(2));
}

TEST(clique, triangle) {
  adjacency_matrix const matrix = {
    bitvec{"110"},
    bitvec{"101"},
    bitvec{"011"},
  };

  const auto cliques = clique_cover(matrix);
  EXPECT_EQ(cliques.size(), 1);
  EXPECT_EQ(cliques[0].count(), 3);
  EXPECT_EQ(cliques[0], bitvec::max(3));
}

TEST(clique, complex) {
  adjacency_matrix const matrix = {
    bitvec{"0101110"},
    bitvec{"0010101"},
    bitvec{"0011011"},
    bitvec{"1110101"},
    bitvec{"1101110"},
    bitvec{"1011001"},
    bitvec{"0111000"},
  };

  const auto cliques = clique_cover(matrix);
  EXPECT_EQ(cliques.size(), 2);
  EXPECT_EQ(cliques[0], bitvec{"1111000"});
  EXPECT_EQ(cliques[1], bitvec{"0000111"});
}