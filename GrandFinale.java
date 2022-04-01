/* I think I used a method similar to Route B but I'm not too sure. I had a good idea of something that would work and did that instead of Route A or B since I wasn't too sure how to implement them.
This program works similarly to my Ex2, in the sense that it creates a path to the end by overwriting the direction the robot leaves a particular junction from every time it goes to that junction.
By the time it reaches the end of the maze, the stored last directions the robot took from every junction makes a path to the end with all the unnecessary extra paths removed due to the overwriting
rule. The difference between this program and Ex2, however, is that this program also works on loopy mazes since it uses Tremaux's Algorithm to explore the maze.  */
/* The program works most of the time when the robot starts in a corridor and has to backtrack back onto the starting point, but sometimes it doesn't and I don't know what causes that to happen. 
Program could be improved by not requiring co-ordinates to store junctions and call for junction information and fixing the corridor issue. */

import uk.ac.warwick.dcs.maze.logic.IRobot;

public class GrandFinale
{
	private byte explorerMode = 1; // 1 = Explore, 0 = Backtrack
	private int pollRun = 0; // Incremented after each pass
	private RobotData robotData; // Data store for junctions
	private int currentJunction = 0;
	private boolean startsinCorridor;

	// Acts as main method
	// Tells program to use exploreControl method or backtrackControl method based on whether robot should be exploring or backtracking, denoted by the explorerMode variable
	public void controlRobot(IRobot robot) {

		// Resets junctions stored in RobotData object if number of moves made by the robot is zero and the number of runs through that particular maze is zero
		if ((robot.getRuns() == 0) && (pollRun == 0)) {
			robotData = new RobotData();
			startsinCorridor = false;
		}

		// Starts robot going East by default so that the robot is always going in the same direction to start with
		// If the robot did the first run and the second run starting with a different heading, it could cause the robot to go to the wrong junctions and mess up the program
		if (pollRun == 0) {
			robot.setHeading(IRobot.EAST);
		}

		if (explorerMode == 1 & robot.getRuns() == 0) {
			exploreControl(robot);	
		}
		else if (explorerMode == 0 & robot.getRuns() == 0) {
			backtrackControl(robot);
		}

		// After the first run, program does the directPath to the end using the stored directions
		if (robot.getRuns() >= 1) {
			directPath(robot);
		}

		// Increments number of turns by 1 after every turn
		pollRun++;
	}

	// Robot takes every possible PASSAGE path
	// If there are multiple PASSAGE paths the robot can take, it randomly chooses between them
	public void exploreControl(IRobot robot) {
		int direction = IRobot.AHEAD;
		byte numExits = nonwallExits(robot);
		int beenbeforeExits = beenbeforeExits(robot);

		// Decides which method to use based on the number of exits around the robot
		switch (numExits) {
		case 4:
		case 3: 
			// Records junction if robot has not been to that junction before
			if (beenbeforeExits <= 1) {
				direction = crossroads(robot);
				robotData.junctionRecorder(robot.getLocation().x, robot.getLocation().y, robot.getHeading(), convertDirectiontoHeading(direction, robot.getHeading()));
			}
			// Robot turns around if it has been to the junction before and starts backtracking
			else if (beenbeforeExits >= 2) {
				direction = IRobot.BEHIND;
				explorerMode = 0;
			}
			break;
		case 2: 
			direction = corridor(robot);
			// If robot starts in a corridor, it treats the starting point as a junction (since the robot has two directions it can move in) and records it as a junction
			if (pollRun == 0) {
				robotData.junctionRecorder(robot.getLocation().x, robot.getLocation().y, robot.getHeading(), convertDirectiontoHeading(direction, robot.getHeading()));
				startsinCorridor = true;
			}
			// If the robot walks back over the starting point
			if (startsinCorridor == true && robot.getLocation().x == 1 && robot.getLocation().y == 1) {
				robotData.updateCorridorJunction(convertDirectiontoHeading(direction, robot.getHeading()));
			}
			break;
		case 1:
			direction = deadend(robot);
			// Robot should not be backtracking in the first move, so it only backtracks from a deadend after the first move
			if (pollRun >= 1) {
				explorerMode = 0;
			}
		}

		robot.face(direction);		
	}

	// Controls robot when it's backtracking through a maze (aka going over BEENBEFORE squares)
	// When it detects a PASSAGE path, it goes back into explore mode
	public void backtrackControl(IRobot robot) {
		int numExits = nonwallExits(robot);
		int direction = IRobot.AHEAD;
		int passageExits = passageExits(robot);
		int currentJunction;

		// Decides what robot should do based on the number of exits around it
		switch (numExits) {
			case 1: 
				direction = deadend(robot);
				break;
			case 2: 
				direction = corridor(robot);
				// If the robot backtracks onto the starting point, it treats it like a junctions and updates the departure heading for it
				if (startsinCorridor == true && robot.getLocation().x == 1 && robot.getLocation().y == 1) {
					robotData.updateCorridorJunction(convertDirectiontoHeading(direction, robot.getHeading()));
					explorerMode = 1;
				}
				break;
			// Robot should do the same thing if it's at a junction (3 exits) or at a crossroads (4 exits)
			case 3:
			case 4: 
				// Finds out junction number of the junction the robot is currently on
				currentJunction = robotData.searchforJunction(robot.getLocation().x, robot.getLocation().y);
				// If there are no PASSAGE paths, robot goes into direction opposite to the one it originally came into the junction with
				if (passageExits == 0) {
					robot.setHeading(robotData.searchforOriginalHeading(currentJunction));
					direction = IRobot.BEHIND;
				}
				// Otherwise, it takes a PASSAGE path and records which direction it went, overwriting the last recorded direction it went from this junction and goes into explore mode
				else {
					direction = crossroads(robot);
					robotData.updateDepartedHeading(convertDirectiontoHeading(direction, robot.getHeading()));
					explorerMode = 1;
				}
		}
		robot.face(direction);
	}

	// Uses stored departure directions to guide robot directly to end of maze
	public void directPath(IRobot robot) {
		int direction = IRobot.AHEAD;
		int numExits = nonwallExits(robot);

		// Decides what robot should do based on the number of exits around the robot
		switch (numExits) {
			case 1:
				direction = deadend(robot);
				robot.face(direction);
				break;
			case 2:
				// If the robot starts in a corridor, it should act as a junction and use the first depature heading
				if (pollRun == 0) {
					robot.setHeading(robotData.searchforDepartedHeading(currentJunction));
					currentJunction++;
				}
				// After first move, robot will act normally in corridors
				else {
					direction = corridor(robot);
					robot.face(direction);
				}
				break;
			// Robot should do same thing if it's at a junctions or crossroads
			case 3:
			case 4:
				// Robot should have the same heading the as the departure heading
				robot.setHeading(robotData.searchforDepartedHeading(currentJunction));
				currentJunction++;
		}
	}

	// Called when the reset button is pressed in the maze environment
	public void reset() {
		robotData.resetJunctionCounter();
		pollRun = 0;
		currentJunction = 0;	
	}

	// Counts number of exits (paths that are not walls) around robot
	private byte nonwallExits(IRobot robot) {
		byte numExits = 0;
		for (int d = IRobot.AHEAD; d <= IRobot.LEFT; d++) {
			// Increments numExits by 1 if the robot detects a path that is not a wall in the direction it's currently looking
			if (robot.look(d) != IRobot.WALL) {
				numExits++;
			}
		}
		return numExits;
	}

	// Returns a direction that isnt't behind and doesn't cause a collision
	private int corridor(IRobot robot) {
		int d;
		// Looks in every direction around the robot
		for (d = IRobot.AHEAD; d <= IRobot.LEFT; d++) {
			// When it finds a direction that isn't behind the robot and doesn't cause a collision, it stops looking around the robot and returns that direction
			if (robot.look(d) != IRobot.WALL & d != IRobot.BEHIND) {
				break;
			}
		}
		return d;
	}

	// Returns a direction that doesn't cause a collision
	private int deadend(IRobot robot) {
		int d;
		// Looks in every direction around the robot
		for (d = IRobot.AHEAD; d <= IRobot.LEFT; d++) {
			// If it looks in a direction which doesn't cause a collision, it stops looking around the robot and returns that direction
			if (robot.look(d) != IRobot.WALL) {
				break;
			}
		}
		return d;
	}

	// Don't need to make a junction method as the crossroads method works for both junctions and crossroads
	private int crossroads(IRobot robot) {
		// Max number of passages is 4, so size of array of PASSAGE paths is 4
		int passages[] = new int[4];
		int n = 0, d = IRobot.AHEAD;
		// Looks in every direction around robot using for loop and stores directions which have PASSAGE exits in an array
		for (int i = IRobot.AHEAD; i <= IRobot.LEFT; i++) {
			if (robot.look(i) == IRobot.PASSAGE) {
				passages[n] = i;
				n++;
			}
		}

		if (n != 0) {
			// Chooses randomly between PASSAGE exits if there are any around the robot
			// Also works in the case where there's only one PASSAGE path
			d = passages[(int) Math.floor(Math.random() * n)];
			return d;
		}
		// If there are no PASSAGE exits around the robot, it chooses a random direction and returns that direction
		else {
			while (robot.look(d) == IRobot.WALL) {
			d = IRobot.AHEAD + (int) Math.floor(Math.random() * 4);
			}
			return d;
		}
	}

	// Returns the number of exits around the robot that are BEENBEFORE paths
	private byte beenbeforeExits(IRobot robot) {
		byte numBeenbeforeExits = 0;
		// Looks in every direction around the robot
		for (int n = IRobot.AHEAD; n <= IRobot.LEFT; n++) {
			// If it looks in a direction which has a BEENBEFORE path, it increments numBeenbeforeExits by 1
			if (robot.look(n) == IRobot.BEENBEFORE) {
				numBeenbeforeExits++;
			}
		}
		return numBeenbeforeExits;
	}

	// Returns the number of exits around the robot that are PASSAGE paths
	private byte passageExits(IRobot robot) {
		byte passageExits = 0;
		// Looks in every direction around the robot
		for (int n = IRobot.AHEAD; n <= IRobot.LEFT; n++) {
			// If it looks in a direction which has a PASSAGE path, it increments passageExits by 1
			if (robot.look(n) == IRobot.PASSAGE) {
				passageExits++;
			}
		}
		return passageExits;
	}

	// Converts a direction in relation to the robots heading to a heading
	private int convertDirectiontoHeading(int wantedDirection, int currentHeading) {
		int difference = wantedDirection - IRobot.AHEAD;
		int wantedHeading;

		if (currentHeading + difference > IRobot.WEST) {
			difference -= 4;
		}

		wantedHeading = currentHeading + difference;
		return wantedHeading;
	}
}


//Used to instantiate an object that stores information about the junctions the robot encounters
class RobotData
{
	private static int maxJunctions = 10000; // Max number junctions likely to occur
	private int junctionCounter; // No. of junctions stored
	private int[] juncX = new int[maxJunctions]; // Stores x co-ordinates of junctions the robot encounters
	private int[] juncY = new int[maxJunctions]; // Stores y co-ordinates of junctions the robot encounters
	private int[] arrived = new int[maxJunctions]; // Stores initial headings the robot came into the junctions with. Useful when backtracking
	private int[] departed = new int[maxJunctions]; // Stores an array of the heading the robot last left a junction with. If robot follows these directions everytime it reaches a junction, it will be led to the end of the maze

	public void RobotData() {
		junctionCounter = 0;
	}

	public void resetJunctionCounter() {
		junctionCounter = 0;
	}

	// Records co-ordinates of junction, heading as robot came into junction and heading as robot leaves junction
	public void junctionRecorder(int xCoord, int yCoord, int arrivalHeading, int departureHeading) {
		juncX[junctionCounter] = xCoord;
		juncY[junctionCounter] = yCoord;
		arrived[junctionCounter] = arrivalHeading;
		departed[junctionCounter] = departureHeading;
		junctionCounter++;
	}

	// Returns junction number of the junction that corresponds to location of robot
	public int searchforJunction(int xCoord, int yCoord) {
		int n = 0;
		for (n = 0; n <= junctionCounter; n++) {
			if (juncX[n] == xCoord & juncY[n] == yCoord) {
				break;
			}
		}
		junctionCounter = n + 1; // Have to do this so when robot encounters next junction, junctionCounter would be the right value
		return n;
	}

	// Returns the heading of the robot as it originally entered the junction
	public int searchforOriginalHeading(int junctionNumber) {
		return arrived[junctionNumber];
	}

	// Overwrites previous departure heading of the robot
	public void updateDepartedHeading(int newDepartureHeading) {
		departed[junctionCounter - 1] = newDepartureHeading; // Have to include the -1 to counteract the +1 from line 320
	}

	// Finds departure heading for a particular junction
	public int searchforDepartedHeading(int junctionNumber) {
		return departed[junctionNumber];
	}

	// Used to update the departure heading of the starting position if the robot starts in a corridor
	public void updateCorridorJunction(int newDepartureHeading) {
		departed[0] = newDepartureHeading;
	}
}