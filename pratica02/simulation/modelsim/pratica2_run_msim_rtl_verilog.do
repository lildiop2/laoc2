transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+C:/Users/gabri/Documents/LAOC2/pratica2 {C:/Users/gabri/Documents/LAOC2/pratica2/pratica2.v}
vlog -vlog01compat -work work +incdir+C:/Users/gabri/Documents/LAOC2/pratica2 {C:/Users/gabri/Documents/LAOC2/pratica2/romLPM2.v}

