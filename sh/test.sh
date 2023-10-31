#!/bin/bash

# declaring
difficulty=$1
start_index=0
end_index=$2
# build
rm -f $1_out/*.txt $1_out/*.png ./$1_out.txt
make clean && make -j16

# parse aruguments
while getopts ":dv" opt; do
  case $opt in
    d)
      draw=true
      ;;
    v)
      verify=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done
verify=true
#draw=true

# run
total_time=0
max_time=0
min_time=10
for i in $(seq $start_index $end_index); do
    echo "Running $1_case test$i.txt"
    echo "Case $i" >> ./$1_out.txt
    t=$(./router ./$1_case/test$i.txt ./$1_out/test$i.txt ./$1_out.txt | grep -o "Time [0-9.]*" | awk '{print $2}')
    total_time=$(echo "$total_time+$t" | bc -l)
done
avg_time=$(echo "$total_time/($end_index+1)" | bc -l)

# verify
if [ "$verify" = true ]; then
    not_pass_files=""
    not_succ_files=""
    for i in $(seq $start_index $end_index); do
        echo "Running case test$i.txt."
        if [ ! -e ./$1_out/test$i.txt ]; then
            not_pass_files="$not_pass_files$i, "
        elif (./verifier ./$1_out/test$i.txt | grep -q "Error"); then
            echo "Error found in case test$i.txt."
            echo "Please run the following code to check the log."
            echo "./verifier ./$1_out/test$i.txt"
            not_succ_files="$not_succ_files$i, "
        else
            echo "Passed."
        fi
    done
    if [ -n "$not_pass_files" ]; then
        echo "Cannot run at test{$not_pass_files}.txt"
    fi
    if [ -n "$not_succ_files" ]; then
        echo "Failed at test{$not_succ_files}.txt"
    fi
fi


# draw
if [ "$draw" = true ]; then
    python3 visual.py --dir ./$1_out
fi

# terminal output
echo "total_time $total_time"
echo "avg_time $avg_time"