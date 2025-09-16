# Code for Reciving via bluetooth
##Code for Server and client to send data via bluetooth is inspried by NeuralNine's youtube video made 8. apr. 2023 
# url: https://www.youtube.com/watch?v=8pMaR-WUc6U

import socket

server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
server.bind(("e8:48:b8:c8:20:00", 5))
server.listen(1)

client, addr = server.accept();
run_is_finished = False;
inizilizing = True;

try:
    while True:
        data = client.recv(1024)
       
        # Extracting data from client
        inputData_table = data.decode('utf-8')
        if(inizilizing):
            inputdata_entries = inputData_table.split("; ")  # Split the input [m, min]

            table_distanceRun=  inputdata_entries[0]  # Extract data for table 1
            table2_timeRun = inputdata_entries[1]  # Extract data for table 2

            # Converting data to lists of lists
            table_distanceRun = list(map(int, table_distanceRun.strip('[').strip(']').split(',')))  # Convert data for table 1 to list
            table2_timeRun = list(map(int, table2_timeRun.strip('[').strip(']').split(',')))  # Convert data for table 2 to list
            inizilizing = False

        if data.decode('utf-8') == "stop" or data.decode('utf-8') == "Stop" : 
            print("Run has been stopped")
            break
        
        print(f"Input from user: Distance: {table_distanceRun} m, Segment: {table2_timeRun} min")
        if(run_is_finished):
            message = "Output of the data that should be presented to the user"
            client.send(message.encode("utf-8"))
except OSError as e:
    pass

client.close()
server.close()
