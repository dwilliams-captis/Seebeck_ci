2025 UPDATE st31
Thermal cycle changed to highest to lowest (60, 50, 40 C)
Added an auto calculation for Seebeck

2025 UPDATE st30
Add a single-button push sequence that will go through multiple thermal cycles (40, 50, 60 C) and auto save when completed. 

2025 UPDATE st29
Updated data file layout to include meta information. 
Changed the default sample name. 
Updated button colors

2025 UPDATE st28
Change the name of the "Temperatures" tab to "Heaters"
Reformat the "Start Temperature Plotting" button


20250918 UPDATE st27
Replace text fields for setpoints with +5 and -5 buttons (rounding to nearest 5 or 10, starting from ambient).
Double the height of the Start Heating and Stop Heating buttons.
Show a text confirmation when a file is saved.
Run gsynch.sh (in the same folder as the script) after saving.

20250830 UPDATE st26
1. New "Temps" Tab
Added as tab #4 in the interface
Contains a dedicated temperature plotting area
2. Temperature Plot Controls
Start/Stop Button:
Blue "Start Temperature Plotting" button that turns red "Stop Temperature Plotting" when active
When stopped, it purges all temperature data and starts fresh

Individual Heater Toggle Buttons:
"Hide/Show Temp 1" button for Heater 1 temperature line
"Hide/Show Temp 2" button for Heater 2 temperature line
These buttons only control visibility - the underlying data is retained

3. Temperature Data Collection
New temp_data[] array stores timestamp, temp1, and temp2 values
Data collection controlled by temp_plotting_active flag
Independent from the voltage plotting system

4. Plot Features
Real-time temperature plotting with time on X-axis (seconds from start)
Red line for Heater 1, Blue line for Heater 2
Legend shows which heaters are currently displayed
Automatic axis labeling and titles
Setpoint reference lines for given setpoints.


5. Data Saving
Temperature data is now saved to a separate CSV file (*_templog.csv)
Contains columns: Time, Temp1, Temp2


20250825 UPDATE st25
Added plotting_active flag: A new boolean variable to control when data collection starts, separate from heating control.
Created "Start Plotting" button: Added a new blue button on the Plot tab next to the existing "Re-Scale" and "Show All" buttons.
Modified data collection logic: Changed the condition in update_loop() from checking heating_active to checking plotting_active for adding data to voltage_data[].
Added start_plotting() method: This method:
Sets plotting_active = True
Clears any existing data in voltage_data[]
Changes the button text to "Plotting Active"
Changes the button color to green
Clears the current plot


20250823 UPDATE st24
1. The contents of tab4 and tab5 should be switched. Tab4 should be where a user saves data and Tab 5 should be a read me file.
2. The “Start Heating” Button should be entirely green with white text but only the outline is showing green and the text is white. 
3. For the text readouts for voltage (in mV) on the plot tab and voltage tab can you add 3 more digits to the readouts?
4. On the voltage tab, can you make the font on the voltage table 24px
5. On the Plot tab,  add a data point to the data points at the bottom of the screen labeled “Seebeck” and show a the slope of the line created for the last 30 data points for voltage and temperature difference?  Use this label and value on the “Voltage Tab” instead of “Voltage/Temp Ratio”?
6. On the plot tab, add a button labeled “Re-Scale” at the top that will re-scale the plot to only show the last 30 data points and rescale the axes accordingly. 
7. On the plot tab, add a button next to the “Re-Scale” button that shows all of the currently tracked data in voltage_data (essentially un-doing the “Re-Scale” button.


