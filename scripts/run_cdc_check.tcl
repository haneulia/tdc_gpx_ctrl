open_project C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.xpr
open_run synth_1 -name synth_1
report_cdc -details -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/synth_cdc.txt
report_clock_interaction -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/synth_clock_interaction.txt
report_methodology -file C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/synth_methodology.txt
puts "CDC / clock / methodology reports written"
close_project
