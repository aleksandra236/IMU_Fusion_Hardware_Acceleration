# =============================================================================
# build_ahrs_project.tcl
# ------------------------------------------------------------------------------
# Automatski gradi kompletan y26-g05 AHRS projekat na Zybo Z7-10 (top_zybo).
#
# Sve putanje su relativne u odnosu na lokaciju skripte. Nista se rucno ne
# konfigurise. Skripta:
#   - kreira Vivado projekat
#   - dodaje sve VHDL fajlove iz hdl/ (iskljucuje behavioralne verzije iz sinteze)
#   - dodaje sve Xilinx IP core-e iz ip/
#   - dodaje sve XDC constraints iz xdc/
#   - pokrece sintezu, implementaciju i generisanje bitstream-a
#   - kopira izlazne fajlove u release folder
#
# Pokretanje iz Vivado 2022.2 GUI-a:
#   Tools -> Run Tcl Script... -> izaberi ovaj fajl
# ILI iz Tcl Console:
#   cd <putanja_do_rtl>/tcl
#   source build_ahrs_project.tcl
# =============================================================================

# -----------------------------------------------------------------------------
# KORAK 1: Parametri i putanje (sve relativno u odnosu na lokaciju skripte)
# -----------------------------------------------------------------------------
set proj_name   "y26-g05"
set part        "xc7z010clg400-1"
set board_part  "digilentinc.com:zybo-z7-10:part0:1.2"
set top_module  "top_zybo"

set scriptDir   [file normalize [file dirname [info script]]]
set rtlDir      [file normalize "$scriptDir/.."]
set projRoot    [file normalize "$rtlDir/.."]

set resultDir   "$projRoot/result/$proj_name"
set releaseDir  "$projRoot/release/$proj_name"

set hdlDir      "$rtlDir/hdl"
set ipDir       "$rtlDir/ip"
set xdcDir      "$rtlDir/xdc"

file mkdir $resultDir
file mkdir $releaseDir

puts ""
puts "=============================================="
puts "=  AHRS Build - $proj_name"
puts "=  Top module: $top_module"
puts "=============================================="
puts "Script dir : $scriptDir"
puts "RTL dir    : $rtlDir"
puts "Result dir : $resultDir"
puts "Release dir: $releaseDir"
puts ""

# -----------------------------------------------------------------------------
# KORAK 2: Zatvori bilo koji vec otvoren projekat (u slucaju da Vivado nije cist)
# -----------------------------------------------------------------------------
catch { close_project }

# -----------------------------------------------------------------------------
# KORAK 3: Kreiranje novog Vivado projekta
# -----------------------------------------------------------------------------
create_project $proj_name $resultDir -part $part -force

set_property board_part         $board_part      [current_project]
set_property target_language    VHDL             [current_project]
set_property default_lib        xil_defaultlib   [current_project]
set_property simulator_language Mixed            [current_project]
set_property enable_vhdl_2008   1                [current_project]

# -----------------------------------------------------------------------------
# KORAK 4: Dodavanje SVIH VHDL fajlova iz hdl/ (VHDL 2008)
# -----------------------------------------------------------------------------
set all_vhd [lsort [glob -nocomplain "$hdlDir/*.vhd"]]
if { [llength $all_vhd] == 0 } {
    error "Nema .vhd fajlova u $hdlDir - proveri strukturu."
}

foreach f $all_vhd {
    add_files -norecurse $f
    set_property file_type {VHDL 2008} [get_files $f]
    puts "VHDL: [file tail $f]"
}

# -----------------------------------------------------------------------------
# KORAK 5: Iskljucivanje behavioralnih verzija iz sinteze
# ------------------------------------------------------------------------------
# ahrs_invSqrt.vhd i ahrs_sec5.vhd su verovatno behavioralne verzije koje
# imaju isti entity name kao _synth pandani, sto bi izazvalo konflikt tokom
# sinteze. Postavljanjem used_in_synthesis=false one ostaju u projektu (mogu
# se koristiti za simulaciju), ali sinteza ih preskace.
# -----------------------------------------------------------------------------
foreach fname {ahrs_invSqrt.vhd ahrs_sec5.vhd} {
    set fpath "$hdlDir/$fname"
    if { [file exists $fpath] } {
        set fobj [get_files $fpath]
        set_property used_in_synthesis false $fobj
        puts "  -> $fname iskljucen iz sinteze (behavioralna verzija)"
    }
}

# -----------------------------------------------------------------------------
# KORAK 6: Dodavanje svih Xilinx IP core-a iz ip/
# -----------------------------------------------------------------------------
set all_xci [lsort [glob -nocomplain "$ipDir/*/*.xci"]]
if { [llength $all_xci] == 0 } {
    error "Nema .xci fajlova u $ipDir - proveri strukturu."
}

foreach xci $all_xci {
    read_ip $xci
    puts "IP:   [file rootname [file tail $xci]]"
}

# Upgrade svih IP-a na trenutnu Vivado verziju
# (kriticno ako su IP-ovi kreirani u starijoj Vivado verziji - inace ostaju
#  zakljucani i sinteza pukne sa "module not found")
puts ""
puts "Upgrade svih IP-a na trenutnu Vivado verziju..."
set locked_ips [get_ips -filter {IS_LOCKED == 1}]
if { [llength $locked_ips] > 0 } {
    puts "  Zakljucani IP-ovi: $locked_ips"
    upgrade_ip $locked_ips
    puts "  Upgrade zavrsen."
} else {
    puts "  Nema zakljucanih IP-a."
}

# Regeneracija IP izlaza za trenutnu Vivado verziju i part
foreach ip [get_ips] {
    catch { generate_target all [get_ips $ip] }
    catch { config_ip_cache -export [get_ips -all $ip] }
}

# -----------------------------------------------------------------------------
# KORAK 7: Dodavanje XDC constraints iz xdc/
# -----------------------------------------------------------------------------
set all_xdc [lsort [glob -nocomplain "$xdcDir/*.xdc"]]
if { [llength $all_xdc] == 0 } {
    puts "WARNING: Nijedan .xdc fajl nije pronadjen u $xdcDir"
} else {
    foreach xdc $all_xdc {
        add_files -fileset constrs_1 -norecurse $xdc
        puts "XDC:  [file tail $xdc]"
    }
}

# -----------------------------------------------------------------------------
# KORAK 8: Postavljanje top modula i update compile order-a
# -----------------------------------------------------------------------------
set_property top $top_module [current_fileset]
update_compile_order -fileset sources_1
puts ""
puts "Top module postavljen na: $top_module"
puts ""

# -----------------------------------------------------------------------------
# KORAK 9: Pre-bitstream skripta (spusta NSTD-1/UCIO-1 na Warning nivo)
# ------------------------------------------------------------------------------
# Ako neki I/O port nije eksplicitno constrained u XDC (nema LOC ili IOSTANDARD),
# Vivado po defaultu prekida generisanje bitstream-a. Ova pre-skripta omogucava
# da se bitstream ipak generise (uz upozorenje). Ovo je isti trik iz Vezbe 13.
# -----------------------------------------------------------------------------
set pre_bit_script "$resultDir/pre_write_bitstream.tcl"
set fp [open $pre_bit_script w]
puts $fp "set_property SEVERITY {Warning} \[get_drc_checks NSTD-1\]"
puts $fp "set_property SEVERITY {Warning} \[get_drc_checks UCIO-1\]"
close $fp
add_files -fileset utils_1 -norecurse $pre_bit_script
set_property STEPS.WRITE_BITSTREAM.TCL.PRE $pre_bit_script [get_runs impl_1]

# -----------------------------------------------------------------------------
# KORAK 10: Sinteza
# -----------------------------------------------------------------------------
puts ""
puts "=============================================="
puts "=  Pokrecem sintezu..."
puts "=============================================="
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

if { [get_property PROGRESS [get_runs synth_1]] != "100%" } {
    error "Sinteza nije zavrsena uspesno. Proveri Vivado log fajl u:\n  [get_property DIRECTORY [get_runs synth_1]]"
}
puts "Sinteza zavrsena uspesno."

# -----------------------------------------------------------------------------
# KORAK 11: Implementacija i generisanje bitstream-a
# -----------------------------------------------------------------------------
puts ""
puts "=============================================="
puts "=  Pokrecem implementaciju i bitstream..."
puts "=============================================="
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

if { [get_property PROGRESS [get_runs impl_1]] != "100%" } {
    error "Implementacija nije zavrsena uspesno. Proveri Vivado log fajl u:\n  [get_property DIRECTORY [get_runs impl_1]]"
}
puts "Implementacija i bitstream zavrseni uspesno."

# -----------------------------------------------------------------------------
# KORAK 12: Kopiranje izlaznih fajlova u release folder
# -----------------------------------------------------------------------------
set impl_dir [get_property DIRECTORY [get_runs impl_1]]
set bit_src  "$impl_dir/${top_module}.bit"
set ltx_src  "$impl_dir/${top_module}.ltx"

if { [file exists $bit_src] } {
    file copy -force $bit_src "$releaseDir/${top_module}.bit"
    puts ""
    puts "Bitstream:  $releaseDir/${top_module}.bit"
} else {
    puts "WARNING: Bit fajl nije pronadjen na ocekivanoj lokaciji: $bit_src"
}

# ILA probes fajl (za debug u Hardware Manager-u) - kopira se ako postoji
if { [file exists $ltx_src] } {
    file copy -force $ltx_src "$releaseDir/${top_module}.ltx"
    puts "ILA probes: $releaseDir/${top_module}.ltx"
}

puts ""
puts "=============================================="
puts "=  BUILD ZAVRSEN USPESNO"
puts "=============================================="
puts ""
