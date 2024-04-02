'use client';

import React, {useEffect, useState} from "react";
import {Button, Select, SelectItem} from "@nextui-org/react";
import {wait} from "next/dist/lib/wait";

async function get(endpoint: string): Promise<string> {
    const res = await fetch('http://10.25.2.2:5807/' + endpoint);
    return await res.text()
}

async function post(endpoint: string, data: string): Promise<void> {
    const xhr = new XMLHttpRequest()
    xhr.open("POST", 'http://10.25.2.2:5807/' + endpoint, true)
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
    const [flywheelState, setFlywheelState] = useState(false)

    useEffect(() => {
        get("get/auto chooser").then(value => {
            setAutos(JSON.parse(value))
            setSelected(JSON.parse(value)["Picker"]["selected"])
        })

        setInterval(() => get("get/loop rate (hz)").then(value => setHz(Number.parseFloat(JSON.parse(value)["Number"]))), 500)
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
          <a>{`Flywheel State: ${flywheelState}`}</a>
          <div className="flex flex-col gap-4 w-full">

              {autos.Picker.options.map((auto, idx) => {
                  return (
                      <Button className="w-72" key={idx} onPress={() => post("set/auto chooser", JSON.stringify({
                          Picker: {
                              options: autos?.Picker.options,
                              selected: idx.toString(),
                          }
                      }))} variant="bordered">{auto}</Button>
                  )
              })}

              <a>{autos.Picker.options.at(autos.Picker.selected)}</a>
          </div>
      </section>
  );
}