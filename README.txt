This is README file for VLSI Testing final project
Team   : 13
Members:<劉旻鑫 b08505049>, <林楷崴 b08505039>, <葉冠宏, R11943113>
Date: 2023/06/15

DIRECTORY:
src/ source C++ codes 
report.pdf
README.txt
======
HOW TO CIMPILE:

Then compile the demo, simply follow the following steps
    under src
	make
======
HOW TO RUN:

	./atpg -tdfatpg -ndet number -compression ../sample_circuits/circuit_name.ckt > ../tdf_patterns/circuit_name.pat

	For example,
	under src
	./atpg -tdfatpg -ndet 8 -compression ../sample_circuits/c432.ckt > ../tdf_patterns/c432.pat
======
OTHER NOTICE:
