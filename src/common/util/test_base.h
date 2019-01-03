//
// Created by huyao on 18-3-6.
// Email: hooyao@sjtu.edu.cn
//

#ifndef COMMON_TEST_BASE_H
#define COMMON_TEST_BASE_H

#include "gtest/gtest.h"
#include "gflags/gflags.h"

#define TMAIN                                              \
  int main(int argc, char** argv) {                       \
    ::testing::InitGoogleTest(&argc, argv);                 \
    ::google::ParseCommandLineFlags(&argc, &argv, true);    \
    return RUN_ALL_TESTS();                                 \
  }

#endif //PROJECT_TEST_BASE_H
