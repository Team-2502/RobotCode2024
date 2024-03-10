import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import {Providers} from "@/app/providers";
import clsx from "clsx";

export const metadata: Metadata = {
  title: "Talon Board",
  description: "Display robot telemetry",
};

export default function RootLayout({
                                       children,
                                   }: {
    children: React.ReactNode;
}) {
    return (
        <html lang="en" suppressHydrationWarning>
        <head />
        <body
            className={clsx(
                "min-h-screen bg-background font-sans antialiased",
            )}
        >
        <Providers themeProps={{ attribute: "class", defaultTheme: "dark" }}>
            <div className="relative flex flex-col h-screen">
                <main className="container mx-auto max-w-7xl flex-grow">
                    {children}
                </main>
            </div>
        </Providers>
        </body>
        </html>
    );
}

