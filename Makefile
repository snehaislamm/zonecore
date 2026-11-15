.PHONY: all build unit sil test tidy trace codegen demo clean

all: test

build:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	cmake --build build -j

codegen:
	python3 tools/dbcgen.py can/zonecore.dbc include/zc/zc_can_db.h src/com/zc_can_db.c

unit: build
	./build/zc_unit_tests

sil: build
	python3 -m pytest tests/sil -q

tidy: build
	clang-tidy -p build --quiet src/*/*.c

trace:
	python3 tools/trace.py --out docs/traceability.md --strict

test: unit sil trace

demo: build
	python3 sim/demo.py

clean:
	rm -rf build
