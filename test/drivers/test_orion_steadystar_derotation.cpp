#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "orion_steadystar_driver.h"
#include <chrono>

class MockOrionDriver : public OrionSteadyStarDriver
{
public:
    std::chrono::steady_clock::time_point mock_now;

    MockOrionDriver()
    {
        setDeviceName("Orion SteadyStar");
        mock_now = std::chrono::steady_clock::now();
    }

    std::chrono::steady_clock::time_point getTime() const override
    {
        return mock_now;
    }

    // Expose protected members for testing
    void publicTimerHit() { TimerHit(); }
    void publicSetSimulation(bool sim) { setSimulation(sim); }
    
    // Helper to sync hardware and property state for clean test start
    void syncState(double angle)
    {
        m_Hardware.syncState(angle);
        GotoRotatorNP[0].setValue(angle);
        GotoRotatorNP.setState(IPS_OK);
        m_DerotationStartAngle = angle;
        m_DerotationStartTime = mock_now;
        m_DerotationGuideOffset = 0;
    }

    // Expose protected members for verification
    OrionSteadyStarHardware& getHardware() { return m_Hardware; }
    double getDerotationStartAngle() { return m_DerotationStartAngle; }
    double getDerotationGuideOffset() { return m_DerotationGuideOffset; }
    std::chrono::steady_clock::time_point getDerotationStartTime() { return m_DerotationStartTime; }
    INDI::PropertyNumber& getGotoRotatorNP() { return GotoRotatorNP; }
    INumberVectorProperty& getDerotateNP() { return DerotateNP; }
    INumberVectorProperty& getGuideNP() { return GuideNP; }
};

class OrionSteadyStarDerotationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        driver = std::make_unique<MockOrionDriver>();
        driver->publicSetSimulation(true);
        driver->initProperties();
        
        // Connect the driver
        driver->setConnected(true);

        // Call updateProperties once to define rotator properties before ISNewSwitch
        driver->updateProperties();
        
        // Enable rotator.
        ISState states[] = {ISS_ON, ISS_OFF};
        char *names[] = {(char*)"ENABLED", (char*)"DISABLED"};
        driver->ISNewSwitch(driver->getDeviceName(), "ROTATOR_ENABLED", states, names, 2);

        // Standardize start state
        driver->syncState(0);
    }

    std::unique_ptr<MockOrionDriver> driver;
};

TEST_F(OrionSteadyStarDerotationTest, TestDriftFreeDerotation)
{
    // 1. Initial state at 0
    driver->syncState(0);

    // 2. Set rate: 15 arcsec/sec
    double rate_values[] = { 15.0 };
    char *rate_names[] = { (char*)"RATE" };
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_DEROTATE", rate_values, rate_names, 1);

    // 3. Threshold for 1 step is angle*29 >= 0.5, so angle >= 0.017241379 deg.
    // Time to reach that angle at 15 arcsec/sec is:
    // t = (0.017241379 * 3600) / 15 = 62.0689655 / 15 = 4.137931 seconds.
    
    // Advance time to 4.0s (should not move yet)
    driver->mock_now += std::chrono::milliseconds(4000);
    driver->publicTimerHit();
    EXPECT_DOUBLE_EQ(driver->getHardware().GetTargetAngle(), 0);

    // Advance time to 4.3s (should have moved 1 step)
    driver->mock_now += std::chrono::milliseconds(300); 
    driver->publicTimerHit();
    EXPECT_NEAR(driver->getHardware().GetTargetAngle(), 1.0/29.0, 0.0001);

    // 4. Simulate 1 hour jump
    driver->mock_now += std::chrono::hours(1);
    driver->publicTimerHit();
    
    double total_elapsed_sec = 3600 + 4.3;
    double expected_angle = (total_elapsed_sec * 15.0) / 3600.0;
    int expected_steps = static_cast<int>(expected_angle * 29.0 + 0.5);
    double expected_quantized = expected_steps / 29.0;

    EXPECT_NEAR(driver->getHardware().GetTargetAngle(), expected_quantized, 0.0001);
}

TEST_F(OrionSteadyStarDerotationTest, TestNegativeRate)
{
    driver->syncState(100);
    
    // Set rate: -15 arcsec/sec
    double rate_values[] = { -15.0 };
    char *rate_names[] = { (char*)"RATE" };
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_DEROTATE", rate_values, rate_names, 1);

    // Advance time by 10 seconds
    driver->mock_now += std::chrono::seconds(10);
    driver->publicTimerHit();

    // Theoretical: 100 - (10 * 15 / 3600) = 100 - 0.0416666 = 99.9583333
    // round(99.9583333 * 29) = round(2898.79) = 2899 steps
    // 2899 / 29 = 99.965517
    EXPECT_NEAR(driver->getHardware().GetTargetAngle(), 99.965517, 0.0001);
}

TEST_F(OrionSteadyStarDerotationTest, TestRotatorGuide)
{
    driver->syncState(0);
    
    // Inject a 0.1 degree guide pulse
    double guide_values[] = { 0.1 };
    char *guide_names[] = { (char*)"DELTA" };
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_GUIDE", guide_values, guide_names, 1);
    
    EXPECT_DOUBLE_EQ(driver->getGuideNP().np[0].value, 0);
    EXPECT_DOUBLE_EQ(driver->getDerotationGuideOffset(), 0.1);
    
    driver->publicTimerHit();
    // Theoretical: 0.1. round(0.1 * 29) = 3 steps. 3/29 = 0.103448
    EXPECT_NEAR(driver->getHardware().GetTargetAngle(), 0.103448, 0.0001);
}

TEST_F(OrionSteadyStarDerotationTest, TestRateChangeDoesNotCauseJump)
{
    driver->syncState(0);

    // 1. Start tracking at 15 arcsec/sec
    double rate_values[] = { 15.0 };
    char *rate_names[] = { (char*)"RATE" };
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_DEROTATE", rate_values, rate_names, 1);
    
    // 2. Advance time and inject a guide pulse (0.1 deg)
    driver->mock_now += std::chrono::minutes(1);
    double guide_values[] = { 0.1 };
    char *guide_names[] = { (char*)"DELTA" };
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_GUIDE", guide_values, guide_names, 1);
    
    driver->publicTimerHit();
    double angle_before = driver->getHardware().GetTargetAngle();

    // 3. Change rate to 30 arcsec/sec
    rate_values[0] = 30.0;
    driver->ISNewNumber(driver->getDeviceName(), "ROTATOR_DEROTATE", rate_values, rate_names, 1);

    // 4. Trigger TimerHit immediately
    driver->publicTimerHit();
    double angle_after = driver->getHardware().GetTargetAngle();

    // The angle should be identical or extremely close (no jump)
    EXPECT_NEAR(angle_after, angle_before, 0.0001);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
