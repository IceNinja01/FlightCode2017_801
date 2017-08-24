package org.usfirst.frc.team801.robot.Utilities;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;


@SuppressWarnings("unused")
public class BufferedWriterFRC extends BufferedWriter
{
	public BufferedWriterFRC(Writer out)
	{
		super(out);
	}
	private ArrayList < String > headLine1 = new ArrayList < String > (); //format specs for importing
	private ArrayList < String > headLine2 = new ArrayList < String > (); //Column name
	private ArrayList < String > groupNumber = new ArrayList < String > (); //Group Number;
	private ArrayList <String> eventString = new ArrayList < String > ();
	private String formatSpec;
	private String newString;
	private int i=0;
	private int j;
	private boolean printSpec=true;

	public void writeHeadLine1()
	{
		int n = headLine1.size();

		for(int i = 0; i <= n - 1; i++)
		{
			try
			{
				write(headLine1.get(i) + " ");
				//			out.close();
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}

	}
//	@SuppressWarnings("resource")
	public void writeHeadLine2()
		{
			int n = headLine2.size();

			for(int i = 0; i <= n - 1; i++)
			{
				try
				{
					write(headLine2.get(i) + "\t");
					//			out.close();
				}
				catch(IOException e)
				{
					e.printStackTrace();
				}
			}

		}
	public void writeGroupNum()
	{
		int n = groupNumber.size();

		for(int i = 0; i <= n - 1; i++)
		{
			try
			{
				write(groupNumber.get(i) + "\t");
				//			out.close();
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}

	}
	
	public void writeFRCValue(String name, double value, String groupNum)
	{
		//TODO: why are we writing new format specs if the healine already contains the column?
		if(printSpec)
		{
			formatSpec = "%f";
			headLine1.add(formatSpec);
			groupNumber.add(groupNum);
		}
		if(!headLine2.contains(name))
		{
			headLine2.add(name);
		}
		try
		{
			write(Double.toString(value) + "\t");
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}

	public void addFRCEvent(String string)
	{
		if(!eventString.contains(string))
		{
			eventString.add(string);
		}

	}
	//Writes the Tele-op or Autnonmous or any other event
	public void writeFRCEvent()
	{
		if(printSpec)
		{
			formatSpec = "%s";
			headLine1.add(formatSpec);
		}

		if(!headLine2.contains("Event"))
		{
			headLine2.add("Event");
		}
		if(i>=1){
			for(int j = 0; j < eventString.size(); j++)
			{
				newString = newString + "$" + eventString.get(j);
			}
				try
				{
					if (newString != null) {
							write(newString);
					}
				}
				catch(IOException e)
				{
					e.printStackTrace();
				}
				eventString.clear();
				newString = "";
		}
		i=i+1;
	}
	public void writeFRCBoolean(String name, boolean value, String groupNum)
	{
		//TODO: why are we writing new format specs if the healine already contains the column?
		if(printSpec)
		{
			formatSpec = "%f";
			headLine1.add(formatSpec);
			groupNumber.add(groupNum);
		}
		if(!headLine2.contains(name))
		{
			headLine2.add(name);
		}
		try
		{
			if(value){
				write(1 + "\t");
				}
			else{
				write(0 + "\t");
				}
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}

	public void writeNewLine()
	{

		try
		{
			write("\r" + "\n");
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}
	public String getHeadLine1()
	{

		return headLine1.toString();
	}
	public String getHeadLine2()
	{

		return headLine2.toString();
	}
	
	public Boolean setPrintSpecFormat(boolean printCmd){
		printSpec = printCmd;
		return printSpec;
	}

}