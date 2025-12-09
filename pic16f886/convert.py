import re
import sys

def convert_mpasm_to_picas(input_filename, output_filename):
    with open(input_filename, 'r') as f:
        lines = f.readlines()

    output = []
    
    # State tracking
    in_cblock = False
    
    # 1. HEADER & CONFIGURATION
    # Replace the old include with the new generic XC include
    output.append("; Converted for PIC-AS (XC8)\n")
    output.append("#include <xc.inc>\n")
    
    # Config bit mapping table (Legacy -> Pragma)
    config_map = {
        "_FOSC_EC": "FOSC = EC",
        "_WDTE_OFF": "WDTE = OFF",
        "_PWRTE_ON": "PWRTE = ON",
        "_MCLRE_ON": "MCLRE = ON",
        "_CP_OFF": "CP = OFF",
        "_CPD_OFF": "CPD = OFF",
        "_BOREN_ON": "BOREN = ON",
        "_IESO_OFF": "IESO = OFF",
        "_FCMEN_OFF": "FCMEN = OFF",
        "_LVP_OFF": "LVP = OFF",
        "_BOR4V_BOR40V": "BOR4V = BOR40V",
        "_WRT_OFF": "WRT = OFF"
    }

    for line in lines:
        stripped = line.strip()
        
        # SKIP: Old includes and processor definitions
        if stripped.lower().startswith("list") or stripped.lower().startswith("#include") or stripped.startswith("__PIC16F88X"):
            continue
            
        # SKIP: Error level suppression (not needed in PIC-AS)
        if stripped.lower().startswith("errorlevel"):
            continue

        # TRANSLATE: __CONFIG directives
        if "__CONFIG" in line:
            # Extract the config bits (everything after the comma or space)
            configs = re.findall(r'(_[A-Z0-9_]+)', line)
            for cfg in configs:
                if cfg in config_map:
                    output.append(f"#pragma config {config_map[cfg]}\n")
            continue

        # TRANSLATE: CBLOCK (Variables) -> PSECT (Sections)
        if stripped.upper().startswith("CBLOCK"):
            # 0x70 is Common RAM (Shared across banks)
            if "0x70" in line or "0X70" in line:
                output.append("\nPSECT udata_shr,class=COMMON,space=1\n")
            else:
                output.append("\nPSECT udata,class=RAM,space=1\n")
            in_cblock = True
            continue

        if stripped.upper() == "ENDC":
            in_cblock = False
            # Switch back to code section after variables
            output.append("\nPSECT code,abs,ovrld\n") 
            continue

        if in_cblock:
            # Handle variable definitions inside CBLOCK
            # Match "VARNAME" or "VARNAME:2"
            match = re.match(r'^([A-Z0-9_]+)(:(\d+))?', stripped.upper())
            if match and not stripped.startswith(";"):
                var_name = match.group(1)
                size = match.group(3) if match.group(3) else "1"
                output.append(f"{var_name}: DS {size}\n")
            continue

        # TRANSLATE: ORG -> Absolute PSECTs
        # PIC-AS handles ORG fine if it is inside an absolute PSECT, 
        # but we ensure the section is declared.
        if stripped.upper().startswith("ORG"):
            # We treat the main code as one big absolute chunk for compatibility
            output.append(line) 
            continue

        # TWEAK: BANKSEL
        # PIC-AS uses BANKMASK() for masking, but BANKSEL usually works. 
        # Sometimes literal handling needs tweaks (0x vs h). 
        # We leave most logic alone as PIC-AS is fairly backward compatible here.
        
        # DEFAULT: Just copy the line
        output.append(line)

    with open(output_filename, 'w') as f:
        f.writelines(output)
    print(f"Conversion complete! Saved to {output_filename}")

# Run the converter
# You would call it like this:
if __name__ == "__main__":
    convert_mpasm_to_picas("exSIDBlaster_single_chip_new.asm", "exSID_picas.s")
    # print("Script ready. Uncomment the function call to run.")