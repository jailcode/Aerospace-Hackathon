### TASK 1: 
    - Something that calcutes magnitude of angular acceleration..., 
    - Something to calculate Kp.., (kp is a constant multiplied by 1 over magnitude)
    - Parsing of values from gyroscope and accerelometer 
    - Function which gives difference in time.. (milisecond)
    - current time (in milisecond) - epoch time.. (basically difference between two inputs) (old time and new time)
    - And something to calculate error function..... (three parameter)
    - something to wait for signals from sensors


### TASK 2:
    - make a function.. which returns an int array of size 3 which is our gyroscope which is probably gonna be your x,y,z..., 
    - we get 2 form of input.., 3 from gyroscope and 3 from accerelometer .... 
    all the inputs are (double) and gyroscope gives angle and accerelometer gives angular acceleration..., 
 
    - we will read input from files... we will have two different files for gyroscope and accerelometer with inputs written inside.... we want to read the components from both files simultaeniously... 


### TASK 3:
    - After getting the input... we move onto the processing stage

    - where we run one cycle of pd.., then we write the output in another file...
    - Output is gimble angels... we only have two gimble angles and one fin angle...
    - Then we repeat whole process while updating the input with expected result from output and added noise



### TASK 4:
    - What exactly we gonna do in processing ????????
        1. we get the expected and measured values from the input we read... (in the beginning measured value would be original value + noise)
        2. calcuating error by subtracting real - expected value.....
         "We do Step 1 and Step 2 twice only in the first loop to get two time stamps for differential"
        3. magnitude we initialize to 1 in the beginning and then we keep updating it......
        4. Note - "The acceleration values only "
    - Running 
