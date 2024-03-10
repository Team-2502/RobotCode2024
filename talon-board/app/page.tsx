'use client';

import React, {useEffect, useState} from "react";
import {Button, Select, SelectItem} from "@nextui-org/react";
import {wait} from "next/dist/lib/wait";

async function get(endpoint: string): Promise<string> {
    const res = await fetch('http://localhost:5807/' + endpoint);
    return await res.text()
}

async function post(endpoint: string, data: string): Promise<void> {
    const xhr = new XMLHttpRequest()
    xhr.open("POST", 'http://localhost:5807/' + endpoint, true)
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

    useEffect(() => {
        get("get/auto chooser").then(value => {
            setAutos(JSON.parse(value))
            setSelected(JSON.parse(value)["Picker"]["selected"])
        })
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
      <section className="flex flex-col items-center justify-center gap-4 py-8 md:py-10">
          <div className="flex flex-col gap-4 w-full">
              {/*<Select
                  label="Auto"
                  variant="bordered"
                  placeholder="Select an auto"
                  selectedKeys={[selected]}
                  className="max-w-xs"
                  onChange={handleChange}
              >
                  {autos?.Picker.options.map((auto, idx) => (
                      <SelectItem key={idx} value={idx}>
                          {auto}
                      </SelectItem>
                  ))}
              </Select>*/}

              {autos.Picker.options.map((auto, idx) => {
                  return (
                      <Button key={idx} onPress={() => post("set/auto chooser", JSON.stringify({
                          Picker: {
                              options: autos?.Picker.options,
                              selected: idx.toString(),
                          }
                      }))} variant="bordered" >{auto}</Button>
                  )
              })}

              <a>{autos.Picker.options.at(autos.Picker.selected)}</a>
          </div>
      </section>
  );
}
