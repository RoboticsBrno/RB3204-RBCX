fixRelativeLinkDocs:
	sed  's/\.\/docs\/media/\.\/media/g'  README.md > docs/README.md
	# sed  's/\.\/docs\/media/\.\/media/g'  README.cs.md > docs/README.cs.md

docs-clone-dependencies:
	@echo "Cloning dependencies..."
	mkdir -p build
	git clone https://github.com/RoboticsBrno/RB3204-RBCX-library.git build/RB3204-RBCX-library/

docs-clean-dependencies:
	@echo "Cleaning docs..."
	rm -rf build

# Docs
docs-build: fixRelativeLinkDocs
	@echo "Building docs..."
	mkdocs build

docs-serve: fixRelativeLinkDocs
	@echo "Serving docs..."
	mkdocs serve

docs-serve-once: fixRelativeLinkDocs
	@echo "Deploying docs..."
	mkdocs serve --no-livereload

docs-deploy: fixRelativeLinkDocs
	@echo "Deploying docs..."
	mkdocs gh-deploy --force