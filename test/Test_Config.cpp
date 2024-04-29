#include "config.h"
#include <gtest/gtest.h>

TEST(Config, LoadGood) { 
    LocalizerConfig config = ParseConfig("test/resources/good_config.json");
    config.print();
}