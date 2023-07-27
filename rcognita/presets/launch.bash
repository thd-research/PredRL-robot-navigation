#! /bin/bash
k=0
radius=5
#for ((radius=1,5; radius<6; radius++))
#do
    for ((number_of_test=0; number_of_test < 1; number_of_test++))
    do
        for ((i=0; i<20; i++))
        do
            echo -e "\033[32m TEST Number: $((k=$k+1)) \033[0m"

            ./main_3wrobot_ros_obst.py --dt 0.05 --Nactor 5 --pred_step_size_multiplier 2 --is_log_data True --ctrl_mode RQL --j $number_of_test --set_radius $radius
            
        done
    done

    for ((number_of_test=0; number_of_test < 1; number_of_test++))
    do
        for ((i=0; i<20; i++))
        do
            echo -e "\033[32m TEST Number: $((k=$k+1)) \033[0m"

            ./main_3wrobot_ros_obst.py --dt 0.05 --Nactor 5 --pred_step_size_multiplier 2 --is_log_data True --ctrl_mode SQL --j $number_of_test --set_radius $radius
            
        done
    done

    for ((number_of_test=0; number_of_test < 1; number_of_test++))
    do
        for ((i=0; i<1000; i++))
        do
            echo -e "\033[32m TEST Number: $((k=$k+1)) \033[0m"

            ./main_3wrobot_ros_obst.py --dt 0.05 --Nactor 5 --pred_step_size_multiplier 2 --is_log_data True --ctrl_mode MPC --j $number_of_test --set_radius $radius

        done
    done
#done
