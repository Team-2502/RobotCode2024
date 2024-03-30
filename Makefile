LIB=RobotCode2024
OUT=target/arm-unknown-linux-gnueabi/release/$(LIB)
DEPLOY=javastub/src/main/deploy/$(LIB)
TEAM=25.02

.PHONY: check
check:
	cargo check

.PHONY: compile
compile: $(OUT)

$(OUT): src/** Cargo.* talon-board/out
	cargo build --release --target arm-unknown-linux-gnueabi

$(dir $(DEPLOY)): 
	mkdir -p $(dir $(DEPLOY))

$(DEPLOY): $(dir $(DEPLOY)) $(OUT)
	cp $(OUT) $(dir $(DEPLOY))

.PHONY: deploy-scp
deploy-scp: $(OUT)
	ssh lvuser@10.$(TEAM).2 /usr/local/frc/bin/frcKillRobot.sh
	scp $(OUT) lvuser@10.$(TEAM).2:

.PHONY: deploy
deploy: $(DEPLOY)
	cd javastub; ./gradlew deploy

.PHONY: win
win:
	cargo build --release --target arm-unknown-linux-gnueabi
	copy target\arm-unknown-linux-gnueabi\release\libRobotCode2024.so javastub\src\main\deploy
	cd javastub && gradlew deploy
