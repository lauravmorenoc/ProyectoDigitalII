transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/uart_tx.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/pixel_catcher.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/pclk_driver.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/main.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/color_finder.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/buffer_ram_dp.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/arduinoUART.v}
vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/image_sender.v}

vlog -vlog01compat -work work +incdir+C:/Users/LAURA/Documents/7mo\ sem/Digital\ II/Proyecto\ final/Camara_mix {C:/Users/LAURA/Documents/7mo sem/Digital II/Proyecto final/Camara_mix/testbench.v}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  testbench

add wave *
view structure
view signals
run -all
