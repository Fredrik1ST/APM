import pandas as pd

def get_elite_speed_schedule(distance, sex):

    '''
    Gets data from elite runners pacing strategy based on sex and length of run.
    Will be implemented in the function calculate_optimal_speed_pattern() that decides the distance based on the total distance chosen by our runner.
    
    Returns a dataframe with columns Speed(m/s) and Length (distance segments in simulator)
    '''

    filename = f"pace_elite_{distance}m_{sex}.xlsx"
    try:
        df = pd.read_excel(filename)
        return df
    except Exception as e:
        print(f"Error occurred while reading {filename}: {e}")
        return None

def calculate_optimal_speed_pattern(distance, sex, finish_time):

    '''
    df_elite = dataframe from get_elite_speed_schedule(). Will be implemented as used in this function later as opposed to now being decided first.
    distance = total distance of race/session. Decided by our runner. [m]
    finish_time = desired finishing time for our runner. [s]
    '''

    df_elite = get_elite_speed_schedule(distance, sex)

    df_adjusted = df_elite.copy()

    v_avg_elite = df_elite['Speed(m/s)'].mean()
    N = len(df_elite)

    v_avg_desired = distance/finish_time

    #optimal_speed_array = pd.Series(index = df_adjusted.index)
    change_speed = abs(v_avg_desired - v_avg_elite)
    #optimal_speed_array.iloc[0] = 0

    for i in range(1, N):
        df_adjusted['Speed(m/s)'].iloc[i] = df_adjusted['Speed(m/s)'].iloc[i] - change_speed

        #optimal_speed = df_adjusted['Speed(m/s)'].iloc[i] - change_speed
        #optimal_speed_array.iloc[i] = optimal_speed
    
    return df_adjusted #Ønsker å returnere dataframe på samme format som df_elite




