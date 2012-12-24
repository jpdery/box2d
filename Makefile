all: build build-compress

build:
	@wrup -r box2d ./ > box2d.js
	@echo "File written to 'box2d.js'"

build-compress:
	@wrup -r box2d ./ > box2d.min.js --compress
	@echo "File written to 'box2d.min.js'"

.PHONY: test