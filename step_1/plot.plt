reset
set title "movement"
set xlabel "X (m)"
set ylabel "Y (m)"
set grid
set size ratio -1
set style fill empty
set object 1 circle at 0,0 size first 6371000
plot "grav.csv" using 1:2 with lines

