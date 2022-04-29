#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

// Phase 1
// Autocomplete function
TEST(TrojanMapStudentTest, Autocomplete) {
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
TEST(TrojanMapStudentTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance("cat", "cut"), 1);
  EXPECT_EQ(m.CalculateEditDistance("sunday", "saturday"), 3);
}

// Test FindClosestName function
TEST(TrojanMapStudentTest, FindClosestName) {
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

// Phase 2
// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapStudentTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  
  // Test from KFC to Ralphs
  auto path = m.CalculateShortestPath_Dijkstra("KFC", "Ralphs");
  std::vector<std::string> gt{"3088547686","4835551100","4835551097","4835551101","4835551096","2613117890","6807554573","6787803640","7298150111","7477947679","6813416163","122814440","7200139036","7882624618","6813416166","6807536642","6807320427","6807536647","6813416171","6813416123","3398621888","6804883324","5690152756","6816193695","3398621887","3398621886","6816193694","6816193693","6816193692","4015442011","6787470576","6816193770","123230412","452688931","452688933","6816193774","123408705","6816193777","452688940","123318563","6813416129","6813416130","7645318201","6813416131","8410938469","6805802087","4380040167","4380040158","4380040154","2578244375"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to KFC
  path = m.CalculateShortestPath_Dijkstra("Ralphs", "KFC");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapStudentTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from KFC to Ralphs
  auto path = m.CalculateShortestPath_Bellman_Ford("KFC", "Ralphs");
  std::vector<std::string> gt{
      "3088547686","4835551100","4835551097","4835551101","4835551096","2613117890","6807554573","6787803640","7298150111","7477947679","6813416163","122814440","7200139036","7882624618","6813416166","6807536642","6807320427","6807536647","6813416171","6813416123","3398621888","6804883324","5690152756","6816193695","3398621887","3398621886","6816193694","6816193693","6816193692","4015442011","6787470576","6816193770","123230412","452688931","452688933","6816193774","123408705","6816193777","452688940","123318563","6813416129","6813416130","7645318201","6813416131","8410938469","6805802087","4380040167","4380040158","4380040154","2578244375"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to KFC
  path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "KFC");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test cycle detection function
TEST(TrojanMapStudentTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3= {-118.320, -118.290, 34.030, 34.010};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, true);

  // Test case 4
  std::vector<double> square4 = {-118.290, -118.289, 34.030, 34.030};
  auto sub4 = m.GetSubgraph(square4);
  bool result4 = m.CycleDetection(sub4, square4);
  EXPECT_EQ(result4, false);
}

// Test TopologicalSort function
TEST(TrojanMapStudentTest, TopologicalSort) {
  TrojanMap m;

  // Case 1: Input using vector
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC","Target"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"},{"Target","Ralphs"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Target", "Ralphs","KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);

  // Case 2: Read input from csv file
  std::string dependencies_filename = "/Users/divya/Documents/final-project-Divya-Nandlal-Sahetya/input/topologicalsort_dependencies_2.csv";
  std::string locations_filename = "/Users/divya/Documents/final-project-Divya-Nandlal-Sahetya/input/topologicalsort_locations_2.csv";
  location_names = m.ReadLocationsFromCSVFile(locations_filename);
  dependencies = m.ReadDependenciesFromCSVFile(dependencies_filename);
  result = m.DeliveringTrojan(location_names, dependencies);
  gt ={"Shell", "Starbucks","Target","Ralphs","Chick-fil-A","Chipotle","KFC"};
  EXPECT_EQ(result, gt);
}

// Phase 3
// Test TSP function


TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  // Example 1 : 7 points
  std::vector<std::string> input{"5237417650","369169803", "9591449465", "5237388100", "5237417652", "5237417654","5231970324" }; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5237417650","369169803","5231970324","5237417654","5237417652","5237388100","9591449465","5237417650"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);

  // Example 2 : 10 points
  std::vector<std::string> input2{"6816180165","8382592030","7396358386","6042989994","6813379544","6808271398","1732243907","6799110618","8345239388","123067111"}; // Input location ids 
  auto result2 = m.TravellingTrojan_Brute_force(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"6816180165","6799110618","7396358386","1732243907","8345239388","6813379544","6808271398","8382592030","123067111","6042989994","6816180165"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);

  // Example 3: 5 points
  std::vector<std::string> input3{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result3 = m.TravellingTrojan_Brute_force(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);

}

TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  // Example 1 : 7 points
  std::vector<std::string> input{"5237417650","369169803", "9591449465", "5237388100", "5237417652", "5237417654","5231970324" }; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5237417650","369169803","5231970324","5237417654","5237417652","5237388100","9591449465","5237417650"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);

  // Example 2 : 10 points
  std::vector<std::string> input2{"6816180165","8382592030","7396358386","6042989994","6813379544","6808271398","1732243907","6799110618","8345239388","123067111"}; // Input location ids
  auto result2 = m.TravellingTrojan_Backtracking(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"6816180165","6799110618","7396358386","1732243907","8345239388","6813379544","6808271398","8382592030","123067111","6042989994","6816180165"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);

  // Example 3: 5 points
  std::vector<std::string> input3{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result3 = m.TravellingTrojan_Backtracking(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);

}

TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  // Example 1 : 7 points
  std::vector<std::string> input{"5237417650","369169803", "9591449465", "5237388100", "5237417652", "5237417654","5231970324" }; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"5237417650","369169803","5231970324","5237417654","5237417652","5237388100","9591449465","5237417650"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);

  // Example 2 : 10 points
  std::vector<std::string> input2{"6816180165","8382592030","7396358386","6042989994","6813379544","6808271398","1732243907","6799110618","8345239388","123067111"}; // Input location ids
  auto result2 = m.TravellingTrojan_2opt(input2);
  std::cout << "My path length: "  << result2.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt2{"6816180165","6799110618","7396358386","1732243907","8345239388","6813379544","6808271398","8382592030","123067111","6042989994","6816180165"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt2) << "miles" << std::endl; // Print the gt path lengths
  bool flag2 = false;
  if (gt2 == result2.second[result2.second.size()-1]) // clockwise
    flag2 = true;
  std::reverse(gt2.begin(),gt2.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt2 == result2.second[result2.second.size()-1]) 
    flag2 = true;
  
  EXPECT_EQ(flag2, true);

  // Example 3: 5 points
  std::vector<std::string> input3{"1836112325","2871078026","1865156358","6808311401","6817165995"}; // Input location ids 
  auto result3 = m.TravellingTrojan_2opt(input3);
  std::cout << "My path length: "  << result3.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt3{"1836112325","1865156358","2871078026","6808311401","6817165995","1836112325"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt3) << "miles" << std::endl; // Print the gt path lengths
  bool flag3 = false;
  if (gt3 == result3.second[result3.second.size()-1]) // clockwise
    flag3 = true;
  std::reverse(gt3.begin(),gt3.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt3 == result3.second[result3.second.size()-1]) 
    flag3 = true;
  
  EXPECT_EQ(flag3, true);

}

// Test FindNearby points
TEST(TrojanMapTest, FindNearby) {
  TrojanMap m;
  
  auto result1 = m.FindNearby("cafe", "Starbucks", 10, 5);
  std::vector<std::string> ans1{ "9307516740", "5261316290", "5567718696", "614990288", "4089614984" };
  EXPECT_EQ(result1, ans1);

  auto result2 = m.FindNearby("supermarket", "Ralphs", 10, 5);
  std::vector<std::string> ans2{ "5237417649", "6045067406", "7158034317" };
  EXPECT_EQ(result2, ans2);

  auto result3 = m.FindNearby("library", "Leavey Library", 10, 5);
  std::vector<std::string> ans3{ "368169842", "3699294513", "368169838", "368169848" };
  EXPECT_EQ(result3, ans3);
  
  }