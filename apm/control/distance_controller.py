'''
Distance control mode is activated when the runner falls behind the pace profile.

In distance control mode, the APM only focuses on maintaining a certain distance 
from the runner, giving them a chance to catch up to the pace profile.

If the runner catches up, the APM switches back to cruise control mode.
Otherwise, it keeps maintaining distance until the runner has completed the run or
it is canceled by the user.

Notes: 

- Relies on the rear camera and body tracking module from the ZED SDK

'''