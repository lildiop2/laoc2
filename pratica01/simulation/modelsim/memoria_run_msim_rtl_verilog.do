transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+D:/Usuários/Abdul\ Kevin\ Alexis/Documents/AOC2/LAB/pratica01 {D:/Usu�rios/Abdul Kevin Alexis/Documents/AOC2/LAB/pratica01/memoria.v}
vlog -vlog01compat -work work +incdir+D:/Usuários/Abdul\ Kevin\ Alexis/Documents/AOC2/LAB/pratica01 {D:/Usu�rios/Abdul Kevin Alexis/Documents/AOC2/LAB/pratica01/ramlpm.v}
vlog -vlog01compat -work work +incdir+D:/Usuários/Abdul\ Kevin\ Alexis/Documents/AOC2/LAB/pratica01 {D:/Usu�rios/Abdul Kevin Alexis/Documents/AOC2/LAB/pratica01/disp7seg.v}

