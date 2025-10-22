
#include "common.h"
#include "test_main.h"

TEST(test_json, handle_inf_nan) {
	auto text = R"([123,-123,Infinity,-Infinity,NaN])";

	json j = json::parse(text);
	EXPECT_EQ(j.at(0), 123);
	EXPECT_EQ(j.at(1), -123);
	EXPECT_TRUE(std::isinf(j.at(2).get<double>()) && j.at(2).get<double>() > 0);
	EXPECT_TRUE(std::isinf(j.at(3).get<double>()) && j.at(3).get<double>() < 0);
	EXPECT_TRUE(std::isnan(j.at(4).get<double>()));
	EXPECT_EQ(j.dump(), text);
}

TEST_MAIN
