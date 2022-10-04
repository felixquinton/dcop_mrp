#! /usr/bin/bash

# ECHO COMMAND
# echo Hello World!

# VARIABLES
# Uppercase by convention
# Letters, numbers underscores
NAME="Felix"
# echo "My name is $NAME"

# USER INPUT
# read -p "Enter your name: " USERNAME
# echo "Hello $USERNAME, nice to meet you!"

# SIMPLE IF STATEMENT
# if [ "$NAME" == "Felix" ]
# then
#   echo "Hello Master."
# fi

# IF-ELSE
# if [ "$NAME" == "Felix" ]
# then
#   echo "Hello Master."
# else
#   echo "Hello dogshit."
# fi

# ELSE-IF
if [ "$NAME" == "Felix" ]
then
  echo "Hello Master."
elif [ "$NAME" == "Jack" ]
then
  echo "Hello retard."
else
  echo "Hello dogshit."
fi

# COMPARISON
NUM1=3
NUM2=5
if [ "$NUM1" -gt "$NUM2" ]
then
  echo "$NUM1 is greater than $NUM2"
 else
  echo "$NUM1 is smaller than $NUM2"
fi

# FILE CONDITIONS
FILE="test.txt"
if [ -f "$FILE" ]
then
  echo "$FILE is a file."
else
  echo "$FILE is not a file."
fi

# -d file  True if the file is a directory
# -e file  True if the file exists (better use -f)
# -f file  True if the provided string is a file
# -g file  True if the group id is set on a file
# -r file  True if the file is readable
# -s file  True if the file has a non-zero size
# -u  True if the user id is set on a file
# -w  True if the file is writable
# -x  Trie if the file is an executable.

# CASE STATEMENT
# read -p "Are you 21 or over? Y/N" ANSWER
# case "$ANSWER" in
#   [yY] | [yY][eE][sS])
#     echo "You can have sex with hookers!"
#     ;;
#   [nN] | [nN][oO])
#     echo "You can have sex with teenagers :D"
#     ;;
#   *)
#     echo "Stop being retarded"
#     ;;
# esac

# SIMPLE FOR LOOP
NAMES="Brad Kevin Alice Mark"
for NAME in $NAMES
  do
    echo "Hello $NAME"
done

# WHILE LOOP

# FUNCTION
function sayHello() {
  echo "Hello World"
}

sayHello

# FUNCTION with PARAMS
function greet() {
  echo "Hello, I am $1 and I am $2"
}

greet "Brad" "36"

# CREATE FOLDER AND WRITE TO A FILE

mkdir hello
touch "hello/world.txt"
echo "Hello World" >> "hello/world.txt"
echo "Created file"

timeout 10s ros2 launch patrol_sim_robots inst_5r_20w_01_launch.py 

