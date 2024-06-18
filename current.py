# -*- coding: utf-8 -*-
"""
Created on Tue Jun 18 12:53:50 2024

@author: andy
"""

import pandas as pd

# load CSV file
df = pd.read_csv('Current-Measurement-CR-2.csv')

# verify the first register od data to assure the file ha sbeen loaded correctly
print(df.head())

# uA to mA
df['Current(mA)'] = df['Current(uA)'] / 1000

# mean current
mean_current_mA = df['Current(mA)'].mean()
print(f'Mean current: {mean_current_mA} mA')

# calculate cycle time in hours
# time is in miliseconds and is the interval between samples
# divide by 3 600 000 to convert ms to hrs
cycle_time_hrs = (df['Timestamp(ms)'].iloc[-1] - df['Timestamp(ms)'].iloc[0]) / 3600000
print(f'Ttotal time: {cycle_time_hrs} hours')

# ASSUMING the cycle repeats once every 24 hours
# calculate the consumption of a daily cycle in mAh
daily_consumption_mAh = mean_current_mA * cycle_time_hrs
print(f'Total consumption: {daily_consumption_mAh} mAh')

# battery capacity in mAh
battery_capacity_mAh = 19000

# calculate battery duration in dayss
battery_life_days = battery_capacity_mAh / daily_consumption_mAh
print(f'Battery life: {battery_life_days} days')
