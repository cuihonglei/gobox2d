package main

import (
	"encoding/json"
	"fmt"
	"os"
)

const fileName string = "settings.ini"

type Settings struct {
	testIndex           int
	windowWidth         int
	windowHeight        int
	hertz               float32
	velocityIterations  int32
	positionIterations  int32
	drawShapes          bool
	drawJoints          bool
	drawAABBs           bool
	drawContactPoints   bool
	drawContactNormals  bool
	drawContactImpulse  bool
	drawFrictionImpulse bool
	drawCOMs            bool
	drawStats           bool
	drawProfile         bool
	enableWarmStarting  bool
	enableContinuous    bool
	enableSubStepping   bool
	enableSleep         bool
	pause               bool
	singleStep          bool
}

func MakeSettings() Settings {
	s := Settings{}
	s.reset()
	return s
}

func (s *Settings) reset() {
	s.testIndex = 0
	s.windowWidth = 1600
	s.windowHeight = 900
	s.hertz = 60.0
	s.velocityIterations = 8
	s.positionIterations = 3
	s.drawShapes = true
	s.drawJoints = true
	s.drawAABBs = false
	s.drawContactPoints = false
	s.drawContactNormals = false
	s.drawContactImpulse = false
	s.drawFrictionImpulse = false
	s.drawCOMs = false
	s.drawStats = false
	s.drawProfile = false
	s.enableWarmStarting = true
	s.enableContinuous = true
	s.enableSubStepping = false
	s.enableSleep = true
	s.pause = false
	s.singleStep = false
}

func (s *Settings) save() {

	file, _ := os.Create(fileName)
	fmt.Fprintf(file, "{\n")
	fmt.Fprintf(file, "  \"testIndex\": %d,\n", s.testIndex)
	fmt.Fprintf(file, "  \"windowWidth\": %d,\n", s.windowWidth)
	fmt.Fprintf(file, "  \"windowHeight\": %d,\n", s.windowHeight)
	fmt.Fprintf(file, "  \"hertz\": %.9g,\n", s.hertz)
	fmt.Fprintf(file, "  \"velocityIterations\": %d,\n", s.velocityIterations)
	fmt.Fprintf(file, "  \"positionIterations\": %d,\n", s.positionIterations)
	fmt.Fprintf(file, "  \"drawShapes\": %t,\n", s.drawShapes)
	fmt.Fprintf(file, "  \"drawJoints\": %t,\n", s.drawJoints)
	fmt.Fprintf(file, "  \"drawAABBs\": %t,\n", s.drawAABBs)
	fmt.Fprintf(file, "  \"drawContactPoints\": %t,\n", s.drawContactPoints)
	fmt.Fprintf(file, "  \"drawContactNormals\": %t,\n", s.drawContactNormals)
	fmt.Fprintf(file, "  \"drawContactImpulse\": %t,\n", s.drawContactImpulse)
	fmt.Fprintf(file, "  \"drawFrictionImpulse\": %t,\n", s.drawFrictionImpulse)
	fmt.Fprintf(file, "  \"drawCOMs\": %t,\n", s.drawCOMs)
	fmt.Fprintf(file, "  \"drawStats\": %t,\n", s.drawStats)
	fmt.Fprintf(file, "  \"drawProfile\": %t,\n", s.drawProfile)
	fmt.Fprintf(file, "  \"enableWarmStarting\": %t,\n", s.enableWarmStarting)
	fmt.Fprintf(file, "  \"enableContinuous\": %t,\n", s.enableContinuous)
	fmt.Fprintf(file, "  \"enableSubStepping\": %t,\n", s.enableSubStepping)
	fmt.Fprintf(file, "  \"enableSleep\": %t\n", s.enableSleep)
	fmt.Fprintf(file, "}\n")
	file.Close()
}

func (s *Settings) load() {

	data, err := os.ReadFile(fileName)
	if err != nil {
		return
	}

	var document interface{}
	err = json.Unmarshal(data, &document)
	if err != nil {
		return
	}

	root := document.(map[string]interface{})
	for fieldName, fieldValue := range root {
		if fieldName == "testIndex" {
			if v, ok := fieldValue.(float64); ok {
				s.testIndex = int(v)
			}
			continue
		}
		if fieldName == "windowWidth" {
			if v, ok := fieldValue.(float64); ok {
				s.windowWidth = int(v)
			}
			continue
		}
		if fieldName == "windowHeight" {
			if v, ok := fieldValue.(float64); ok {
				s.windowHeight = int(v)
			}
			continue
		}
		if fieldName == "hertz" {
			if v, ok := fieldValue.(float64); ok {
				s.hertz = float32(v)
			}
			continue
		}
		if fieldName == "velocityIterations" {
			if v, ok := fieldValue.(float64); ok {
				s.velocityIterations = int32(v)
			}
			continue
		}
		if fieldName == "positionIterations" {
			if v, ok := fieldValue.(float64); ok {
				s.positionIterations = int32(v)
			}
			continue
		}
		if fieldName == "drawShapes" {
			if v, ok := fieldValue.(bool); ok {
				s.drawShapes = v
			}
			continue
		}
	}
}
