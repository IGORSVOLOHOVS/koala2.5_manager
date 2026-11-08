// GoogleTest cases for process_param, the one function of the Koala library that needs no robot.
// Each answer below has the shape koala_robot.c hands over: the line after its two-character prefix.

#include "koala_parse.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

namespace {

constexpr int untouched = -12345;

// process_param writes into its source through strtok, so each case parses its own copy.
int parse(const std::string &answer, std::array<int, 6> &values, int fields) {
    std::vector<char> source(answer.begin(), answer.end());
    source.push_back('\0');
    return process_param(source.data(), values.data(), fields);
}

std::array<int, 6> fresh() {
    std::array<int, 6> values{};
    values.fill(untouched);
    return values;
}

}  // namespace

TEST(ProcessParam, ReadsEveryFieldAsked) {
    auto values = fresh();
    EXPECT_EQ(parse("1,2,3", values, 3), 0);
    EXPECT_EQ(values[0], 1);
    EXPECT_EQ(values[1], 2);
    EXPECT_EQ(values[2], 3);
    EXPECT_EQ(values[3], untouched);
}

TEST(ProcessParam, StopsAtTheNumberOfFieldsAsked) {
    auto values = fresh();
    EXPECT_EQ(parse("10,20,30,40", values, 2), 0);
    EXPECT_EQ(values[0], 10);
    EXPECT_EQ(values[1], 20);
    EXPECT_EQ(values[2], untouched);
}

TEST(ProcessParam, ReadsNegativeValuesAndSkipsLeadingSpaces) {
    auto values = fresh();
    EXPECT_EQ(parse("-5, 7,-300", values, 3), 0);
    EXPECT_EQ(values[0], -5);
    EXPECT_EQ(values[1], 7);
    EXPECT_EQ(values[2], -300);
}

TEST(ProcessParam, ReadsALineAsTheSerialPortHandsItOver) {
    // An answer line ends in a carriage return and a line feed: sscanf stops at them.
    auto values = fresh();
    EXPECT_EQ(parse("120,-45\r\n", values, 2), 0);
    EXPECT_EQ(values[0], 120);
    EXPECT_EQ(values[1], -45);
}

TEST(ProcessParam, FailsOnAFieldThatIsNoNumber) {
    auto values = fresh();
    EXPECT_EQ(parse("1,x,3", values, 3), -1);
    EXPECT_EQ(values[0], 1);
    EXPECT_EQ(values[2], untouched);
}

TEST(ProcessParam, TakesTheLeadingNumberOfAMixedField) {
    auto values = fresh();
    EXPECT_EQ(parse("12abc,4", values, 2), 0);
    EXPECT_EQ(values[0], 12);
    EXPECT_EQ(values[1], 4);
}

TEST(ProcessParam, ReadsRunsOfDelimitersAsOne) {
    auto values = fresh();
    EXPECT_EQ(parse(",,1,,2,", values, 2), 0);
    EXPECT_EQ(values[0], 1);
    EXPECT_EQ(values[1], 2);
}

TEST(ProcessParam, ReportsNoErrorOnAShortAnswer) {
    // The original behaviour, kept: fewer fields than asked is still 0, and the rest stay as they were.
    auto values = fresh();
    EXPECT_EQ(parse("4,5", values, 4), 0);
    EXPECT_EQ(values[0], 4);
    EXPECT_EQ(values[1], 5);
    EXPECT_EQ(values[2], untouched);
    EXPECT_EQ(values[3], untouched);
}

TEST(ProcessParam, WritesNothingForAnEmptyAnswer) {
    auto values = fresh();
    EXPECT_EQ(parse("", values, 3), 0);
    EXPECT_EQ(values[0], untouched);
}

TEST(ProcessParam, ReadsNineUltrasonicSensors) {
    // koala_read_us_sensors asks for KOALA_US_SENSORS_NUMBER fields, nine.
    std::array<int, 9> values{};
    std::string answer = "1000,25,300,40,5,600,70,800,9";
    std::vector<char> source(answer.begin(), answer.end());
    source.push_back('\0');
    EXPECT_EQ(process_param(source.data(), values.data(), 9), 0);
    EXPECT_EQ(values[0], 1000);
    EXPECT_EQ(values[4], 5);
    EXPECT_EQ(values[8], 9);
}

TEST(ProcessParam, SplitsTheSourceInPlace) {
    // strtok writes a terminator over each delimiter it passes: callers lose their buffer's text.
    std::array<char, 6> source{'7', ',', '8', ',', '9', '\0'};
    std::array<int, 3> values{};
    EXPECT_EQ(process_param(source.data(), values.data(), 3), 0);
    EXPECT_EQ(source[1], '\0');
    EXPECT_EQ(source[3], '\0');
}
