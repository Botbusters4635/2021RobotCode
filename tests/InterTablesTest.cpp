//
// Created by andrew on 08/11/202
//

#include "gtest/gtest.h"
#include "Math/SystemInterpolatingTable//InterpolatingTable.h"

TEST(GettingTableValues, returnExistingTableValues) {

    InterpolatingTable table;
    for (double i = 0; i < 6; i++) {
        table.setTableData(i, 2.0 * i);
    }

    EXPECT_EQ(table.getIntVel(1.0), 2.0);
    ASSERT_EQ(table.getIntVel(1.0), 2.0);

    EXPECT_EQ(table.getIntVel(2.0), 4.0);
    ASSERT_EQ(table.getIntVel(2.0), 4.0);

    EXPECT_EQ(table.getIntVel(3.0), 6.0);
    ASSERT_EQ(table.getIntVel(3.0), 6.0);

    EXPECT_EQ(table.getIntVel(4.0), 8.0);
    ASSERT_EQ(table.getIntVel(4.0), 8.0);

    EXPECT_EQ(table.getIntVel(5.0), 10.0);
    ASSERT_EQ(table.getIntVel(5.0), 10.0);

}


TEST(InterpolatingValues, simplePredictions) {

    InterpolatingTable table;
    for (double i = 0; i < 6; i++) {
        table.setTableData(i, 2.0 * i);
    }

    EXPECT_EQ(table.getIntVel(1.5), 3.0);
    ASSERT_EQ(table.getIntVel(1.5), 3.0);

    EXPECT_EQ(table.getIntVel(2.5), 5.0);
    ASSERT_EQ(table.getIntVel(2.5), 5.0);

    EXPECT_EQ(table.getIntVel(3.5), 7.0);
    ASSERT_EQ(table.getIntVel(3.5), 7.0);

    EXPECT_EQ(table.getIntVel(4.5), 9.0);
    ASSERT_EQ(table.getIntVel(4.5), 9.0);

    EXPECT_EQ(table.getIntVel(4.1), 8.2);
    ASSERT_EQ(table.getIntVel(4.1), 8.2);

}

TEST(InterpolatingNonLinearValues, returnPredictionOfNonLinearValues){

    InterpolatingTable table;

    table.setTableData(0.0,0.0);
    table.setTableData(11.0,16.0);
    table.setTableData(1.0,4.0);
    table.setTableData(5.0,11.0);
    table.setTableData(4.0,7.0);

    EXPECT_NEAR(table.getIntVel(1.1),4.1,0.01);
    ASSERT_NEAR(table.getIntVel(1.1),4.1,0.01);

    EXPECT_NEAR(table.getIntVel(3.1),6.1,0.01);
    ASSERT_NEAR(table.getIntVel(3.1),6.1,0.01);

    EXPECT_NEAR(table.getIntVel(4.3),8.2,0.01);
    ASSERT_NEAR(table.getIntVel(4.3),8.2,0.01);

    EXPECT_NEAR(table.getIntVel(7.5),13.083,0.01);
    ASSERT_NEAR(table.getIntVel(7.5),13.083,0.01);

}