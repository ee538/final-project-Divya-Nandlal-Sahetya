#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

// Autocomplete function
TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Sh");
  std::unordered_set<std::string> gt = {"Shall Gas", "Shell"}; // groundtruth for "Sh"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("sh");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("sH"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("SH"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());

  // space
  names = m.Autocomplete("Chev");
  gt = {"Chevron 1", "Chevron 2","Chevron"}; // groundtruth for "Chev"
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());

  // Test for leading space
  gt = {"Chevron 1", "Chevron 2"};
  names = m.Autocomplete("Chevron "); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}


// Test FindPosition function
TEST(TrojanMapStudentTest, FindPosition) {
  TrojanMap m;
  
  // Test The Sonshine Shop Thrift Store
  auto position = m.GetPosition("The Sonshine Shop Thrift Store");
  std::pair<double, double> gt1(34.0383026, -118.2917671);
  EXPECT_EQ(position, gt1);

  // Test Newman Recital Hall in Hancock Foundation
  position = m.GetPosition("Newman Recital Hall in Hancock Foundation");
  std::pair<double, double> gt2(34.0197048, -118.2846227);
  EXPECT_EQ(position, gt2);

  // Test Just Ride LA
  position = m.GetPosition("Just Ride LA");
  std::pair<double, double> gt3(34.0339943, -118.2648737);
  EXPECT_EQ(position, gt3);

  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);

  // Test Chipotle
  position = m.GetPosition("Chipotle");
  std::pair<double, double> gt5(34.0169985, -118.2822768);
  EXPECT_EQ(position, gt5);
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance("cat", "cut"), 1);
  EXPECT_EQ(m.CalculateEditDistance("sunday", "saturday"), 3);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  // Example 1
  // Test the same case
  std::string r1 = "Starbucks";
  EXPECT_EQ(m.FindClosestName("Storbucks"), r1);
  EXPECT_EQ(m.FindClosestName("Stooooorbucks"), r1);
  EXPECT_EQ(m.FindClosestName(" Storrrrrrrbucks"), r1);

  // Example 2
  std::string r2 = "Chevron";
  EXPECT_EQ(m.FindClosestName("Chevrrron"), r2);

  // Example 3
  std::string r3 = "CAVA";
  EXPECT_EQ(m.FindClosestName("CAVAS"), r3);

  // Example 4
  std::string r4 = "Just Ride LA";
  EXPECT_EQ(m.FindClosestName("JustRideLA"), r4);
}