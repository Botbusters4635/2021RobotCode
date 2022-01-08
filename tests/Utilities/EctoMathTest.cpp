//
// Created by Abiel on 1/31/19.
//

#include "gtest/gtest.h"

#include "Math/EctoMath.h"

TEST(EctoMath, radiansToDegrees) {
	EXPECT_EQ(EctoMath::radiansToDegrees(0), 0.0);
	EXPECT_EQ(EctoMath::radiansToDegrees(M_PI / 2), 90.0);
	EXPECT_EQ(EctoMath::radiansToDegrees(M_PI), 180.0);
	EXPECT_EQ(EctoMath::radiansToDegrees(2.0 * M_PI), 360.0);
}

TEST(EctoMath, degreesToRadians) {
	EXPECT_EQ(0.0, EctoMath::degreesToRadians(0.0));
	EXPECT_EQ(M_PI / 2.0, EctoMath::degreesToRadians(90.0));
	EXPECT_EQ(M_PI, EctoMath::degreesToRadians(180.0));
	EXPECT_EQ(M_PI * 2.0, EctoMath::degreesToRadians(360.0));
}

TEST(EctoMath, sinc){
	EXPECT_EQ(1.0, EctoMath::sinc(0.0));
	EXPECT_EQ(std::sin(M_PI) / M_PI, EctoMath::sinc(M_PI));
}
