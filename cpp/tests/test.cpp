#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <vector>

#include "mstar_type_defs.hpp"
#include "policy.hpp"
#include "col_set.hpp"
#include "od_mstar.hpp"

// Anonymous namespaces effectively limit scope to the current file
namespace {

  // Tests things that aren't large enought to justify a seperate case
  class UtilTest : public ::testing::Test{
  };


  TEST_F(UtilTest, TestOdCoordHash){
    std::hash<mstar::OdCoord> hasher;
    ASSERT_EQ(hasher(mstar::OdCoord({0, 1}, {0})),
	      hasher(mstar::OdCoord({0, 1}, {0})));
    ASSERT_EQ(hasher(mstar::OdCoord({}, {})),
	      hasher(mstar::OdCoord({}, {})));
    ASSERT_EQ(hasher(mstar::OdCoord({1}, {})),
	      hasher(mstar::OdCoord({1}, {})));


    ASSERT_NE(hasher(mstar::OdCoord({0, 1}, {0})),
	      hasher(mstar::OdCoord({0, 1}, {0,2})));
  }


// // The fixture for testing class Foo.
// class FooTest : public ::testing::Test {
//  protected:
//   // You can remove any or all of the following functions if its body
//   // is empty.

//   FooTest() {
//     // You can do set-up work for each test here.
//   }

//   virtual ~FooTest() {
//     // You can do clean-up work that doesn't throw exceptions here.
//   }

//   // If the constructor and destructor are not enough for setting up
//   // and cleaning up each test, you can define the following methods:

//   virtual void SetUp() {
//     // Code here will be called immediately after the constructor (right
//     // before each test).
//   }

//   virtual void TearDown() {
//     // Code here will be called immediately after each test (right
//     // before the destructor).
//   }

//   // Objects declared here can be used by all tests in the test case for Foo.
// };

// // Tests that the Foo::Bar() method does Abc.
// TEST_F(FooTest, MethodBarDoesAbc) {
//   const string input_filepath = "this/package/testdata/myinputfile.dat";
//   const string output_filepath = "this/package/testdata/myoutputfile.dat";
//   Foo f;
//   EXPECT_EQ(0, f.Bar(input_filepath, output_filepath));
// }

// // Tests that Foo does Xyz.
// TEST_F(FooTest, DoesXyz) {
//   // Exercises the Xyz feature of Foo.
// }

  // TEST(ExampleTest, HelloWorld){
  //   std::cout << "Hello world" << std::endl;
  //   ASSERT_TRUE(true);
  // }

  // TEST(ExampleTest, Math){
  //   ASSERT_GT(3+3, 4);
  // }

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
