package org.usfirst.frc.team5465.robot;

public class AutoDrive
{
	private String[] finstructions;
	private RobotDrive drive;
	private int count ;
	private double distance;

	public AutoDrive(RobotDrive drive)
	{
		finstructions = RobotMap.autoinstructions.split("\\s+");
		this.drive = drive;
	}

	public void autoInit()
	{
		count = 0;
	}

	public String[] configPath(String a)
	{
		String[] result;
		if(a.length() != 4) result = new String[]{};
		else
		{
			if(a.charAt(0) == 'L')
			{
				//DoSwitch
				if(a.charAt(1) == 'T')
				{
					if(a.charAt(2) == 'L') result = new String[]{"F168.0", "T180.0", "F41.0"};
					else if(a.charAt(2) == 'R')result = new String[]{"F52.0", "T168.87:", "F191.61", "T90.0", "F39.0"};
					else result = new String[]{};
				}
				else
				{
					if(a.charAt(2) == 'L') result = new String[]{"F176.0", "T107.54", "F121.13"};
					else if(a.charAt(2) == 'R')result = new String[]{"F235.5", "T180.0", "F211.0", "T90.0", "F57.0"};
					else result = new String[]{};
				}
			}
			else if(a.charAt(0) == 'M')
			{
				if(a.charAt(1) == 'T')
				{
					if(a.charAt(2) == 'L') result = new String[]{"F61.0", "T58.35", "F42.88", "T90.0", "F22.5"};
					else if(a.charAt(2) == 'R')result = new String[]{"F58.0", "T162.18", "F88.23", "T93.42", "F33.56"};
					else result = new String[]{};
				}
				else
				{
					if(a.charAt(2) == 'L') result = new String[]{"F70.5", "T41.87", "F110.12", "T101.83", "F141.5"};
					else result = new String[]{};
				}
			}
			else if(a.charAt(0) == 'R')
			{
				if(a.charAt(1) == 'T')
				{
					if(a.charAt(2) == 'L') result = new String[]{"F168.0", "T180.0", "F41.0"};
					else if(a.charAt(2) == 'R')result = new String[]{"F52.0", "T168.87:", "F191.61", "T90.0", "F39.0"};
					else result = new String[]{};
				}
				else
				{
					if(a.charAt(2) == 'L') result = new String[]{"F176.0", "T107.54", "F121.13"};
					else if(a.charAt(2) == 'R')result = new String[]{"F235.5", "T180.0", "F211.0", "T90.0", "F57.0"};
					else result = new String[]{};
				}
			}
			else
			{
				result = new String[]{};
			}
		}
		return result;
	}

	public void autoPeriodic()
	{
		if(count < finstructions.length)
		{
			String task = finstructions[count];
			char key = task.charAt(0);
			double val = Double.parseDouble(task.substring(1, task.length()));
			boolean done = false;

			if(key == 'F')
			{
				done = drive.autoDrive(val);
			}
			else if(key == 'T')
			{
				done = drive.autoTurn(val%360);
			}
			else
			{
				drive.set(0.0, 0.0);
			}

			if(done)
			{
				count++;
				drive.resetEncoders();
			}
		}
		else
		{
			drive.set(0.0, 0.0);
		}
	}
}
