set xlabel "time"
set ylabel "diff"

file1 = "oct_data.dat"
file2 = "pcd_data.dat"

plot file1 using 1:2 with line title "octo", \
     file2 using 1:2 with line title "pcd"
