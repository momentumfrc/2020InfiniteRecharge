package frc.robot.commands;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;
import java.util.HashSet;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.conditioners.DriveConditioner;

public class DriveCommandTest {

  private class MockDriveSubsystem extends DriveSubsystem {

    private double moveRequest;
    private double turnRequest;

    @Override
    public void stop() {
      moveRequest = 0;
      turnRequest = 0;
    }

    @Override
    public void drive(double moveRequest, double turnRequest) {
      this.moveRequest = moveRequest;
      this.turnRequest = turnRequest;
    }

    public double getMoveRequest() {
      return moveRequest;
    }

    public double getTurnRequest() {
      return turnRequest;
    }
  };

  private MockDriveSubsystem mockDrive = new MockDriveSubsystem();

  private class MockConditioner implements DriveConditioner {
    private double moveScale = 1;
    private double turnScale = 1;

    @Override
    public double conditionMove(double moveRequest) {
      return moveScale * moveRequest;
    }

    @Override
    public double conditionTurn(double turnRequest) {
      return turnScale * turnRequest;
    }

    public double getMoveScale() {
      return moveScale;
    }

    public double getTurnScale() {
      return turnScale;
    }

    public void setMoveScale(double moveScale) {
      this.moveScale = moveScale;
    }

    public void setTurnScale(double turnScale) {
      this.turnScale = turnScale;
    }

  }

  private MockConditioner mockConditioner = new MockConditioner();

  private class MockDriveController implements DriveController {

    private double moveRequest;
    private double turnRequest;

    @Override
    public double getTurnRequest() {
      return turnRequest;
    }

    @Override
    public double getMoveRequest() {
      return moveRequest;
    }

    public void setMoveRequest(double moveRequest) {
      this.moveRequest = moveRequest;
    }

    public void setTurnRequest(double turnRequest) {
      this.turnRequest = turnRequest;
    }

    public void setMTR(double moveRequest, double turnRequest) {
      this.setMoveRequest(moveRequest);
      this.setTurnRequest(turnRequest);
    }

    @Override
    public double getLeftStick() {
      // TODO: implement real tests
      return 0;
    }

    @Override
    public double getRightStick() {
      // TODO: implement real tests
      return 0;
    }
  };

  private MockDriveController mockController = new MockDriveController();

  private DriveCommand command = new DriveCommand(mockDrive, mockController, mockConditioner);

  private static final double ASSERT_DELTA = 1e-5d;

  @Test
  public void testRequirements() {
    assertEquals(command.getRequirements(), new HashSet<Subsystem>(Arrays.asList(mockDrive)));
  }

  @Test
  public void testInitialize() {
    mockDrive.drive(1, 1);
    command.initialize();
    assertEquals(0, mockDrive.getMoveRequest(), ASSERT_DELTA);
    assertEquals(0, mockDrive.getTurnRequest(), ASSERT_DELTA);
  }

  @Test
  public void testExecute() {
    mockConditioner.setMoveScale(0.5);
    mockConditioner.setTurnScale(0.5);

    command.setTankDriveEnabled(false);

    mockController.setMTR(0, 0);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(0.6, 0.2);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(-1, 1);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(-0.2, -0.2);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockConditioner.setMoveScale(-0.8);
    mockConditioner.setTurnScale(-0.1);

    mockController.setMTR(0, 0);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(0.6, 0.2);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(-1, 1);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);

    mockController.setMTR(-0.2, -0.2);
    command.execute();
    assertEquals(mockController.getMoveRequest() * mockConditioner.getMoveScale(), mockDrive.getMoveRequest(),
        ASSERT_DELTA);
    assertEquals(mockController.getTurnRequest() * mockConditioner.getTurnScale(), mockDrive.getTurnRequest(),
        ASSERT_DELTA);
  }

  @Test
  public void testIsFinished() {
    assertEquals(false, command.isFinished());
  }

  @Test
  public void testEnd() {
    mockDrive.drive(1, 1);
    command.end(false);
    assertEquals(0, mockDrive.getMoveRequest(), ASSERT_DELTA);
    assertEquals(0, mockDrive.getTurnRequest(), ASSERT_DELTA);
  }
}