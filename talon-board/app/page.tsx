'use client';

import React, {useEffect, useState} from "react";
import {Button, Select, SelectItem} from "@nextui-org/react";
import {wait} from "next/dist/lib/wait";

async function get(endpoint: string): Promise<string> {
    const res = await fetch('/' + endpoint);
    return await res.text()
}

async function post(endpoint: string, data: string): Promise<void> {
    const xhr = new XMLHttpRequest()
    xhr.open("POST", '/' + endpoint, true)
    xhr.setRequestHeader('Content-Type', 'application/json')
    xhr.send(data)
}

export interface Auto {
    Picker: {
        options: string[],
        selected: number
    }
}

export default function Home() {
    const [autos, setAutos] = useState<Auto | null>(null)
    const [selected, setSelected] = useState(0)
    const [hz, setHz] = useState(0)
    const [load, setLoad] = useState(0)
    const [flywheelState, setFlywheelState] = useState(false)

    useEffect(() => {
        setInterval(() => get("get/auto chooser").then(value => {
            setAutos(JSON.parse(value))
            setSelected(JSON.parse(value)["Picker"]["selected"])
        }), 750)

        setInterval(() => get("get/loop rate (hz)").then(value => setHz(Number.parseFloat(JSON.parse(value)["Number"]))), 500)
        setInterval(() => get("get/rio load").then(value => setLoad(Number.parseFloat(JSON.parse(value)["Number"]))), 200)
        setInterval(() => get("get/flywheel state").then(value => {
            setFlywheelState(JSON.parse(value)["Bool"]);
        }), 250)
    }, []);

    //@ts-ignore
    function handleChange(e) {
        post("set/auto chooser", JSON.stringify(autos))

        //@ts-ignore
        setAutos(prevState => {
            if (!prevState) {return null}

            return {
                Picker: {
                    options: prevState.Picker.options,
                    selected: e.target.value
                }
            }
        })

        console.log(e)

        setSelected(e.target.value)
  }

  if (!autos) return (<></>);

  return (
      <section style={{background: flywheelState ? "green" : "red"}} className="flex flex-col items-center justify-center gap-4 py-8 md:py-10">
          <a>{"Hz: " + hz.toFixed(2)}</a>
          <div style={{width: "75%", background: "black", height: "30px", borderRadius: "5px"}}> 
              <div style={{width: Math.min(load*100, 100)+"%", height: "100%", background: "lightgreen", borderRadius: "5px", transitionDuration: "0.8s", transitionProperty: "width"}}></div> 
          </div>
          <a>{`Flywheel State: ${flywheelState}`}</a>
          <div className="flex flex-col gap-4 w-full">

              {autos.Picker.options.map((auto, idx) => {
                  return (
                      <Button className="w-72" key={idx} onPress={() => post("set/auto chooser", JSON.stringify({
                          Picker: {
                              options: autos?.Picker.options,
                              selected: idx.toString(),
                          }
                      }))} variant="solid" style={(autos.Picker.selected == idx) ? {background:  "green"}: {}}>{auto}</Button>
                  )
              })}
          </div>
      </section>
  );
}
