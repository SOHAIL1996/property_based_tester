./atg.sh 
python3 -m pytest --alluredir=results nav_test.py -v -s
roslaunch jackal_navigation odom_navigation_demo.launch 
allure serve results/