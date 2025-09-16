# Code for client, this code is made for testing
# should be on an external computer
#
# Code for Server and client to send data via bluetooth is inspried by NeuralNine's youtube video made 8. apr. 2023 
# url: https://www.youtube.com/watch?v=8pMaR-WUc6U


import socket

client  = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
client.connect(("e8:48:b8:c8:20:00", 5))
start_Communication = False
correct_input = False
try:
    while(not correct_input):
        input_run  = input("Press 1 for Steady Pace and 2 for User-defined segments: ")
        match input_run:
            case "1":
                input_run_dist =  int(input("Enter running distance: "))
                input_finish_time =  int(input("Enter finish time: "))
                if input_run_dist != "" and input_finish_time != "":
                    correct_input = True
                    print("Input ok")
                    message = str(input_run_dist) + "; " + str(input_finish_time)
                    client.send(message.encode("utf-8"))
                    start_Communication = True
                else:
                    print("Error in input, please try again")
                
            case "2":   
                input_run_dist = input("Enter distances of the segments separated by space: ")
                input_finish_time = input("Enter time of the segments separated by space: ")
                input_run_dist = list(map(int, input_run_dist.split()))  
                input_finish_time = list(map(int, input_finish_time.split())) 

                if len(input_run_dist) == len(input_finish_time) and input_run_dist != "" and input_finish_time != "":
                    correct_input = True
                    print("Input ok")
                    message = str(input_run_dist) + "; " + str(input_finish_time)
                    client.send(message.encode("utf-8"))
                    start_Communication = True
                else:
                    print("Error in input, please try again")
    
    while start_Communication:       
        message = input("Enter 'stop' to stop car: ")
        #if message == 'stop':
        #   break
        client.send(message.encode("utf-8"))
        data = client.recv(10242)
        if not data:
            break
        print(f"Response: {data.decode('utf-8')}")

except OSError as e:
    pass

client.close()