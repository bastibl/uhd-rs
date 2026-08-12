import init, { B2xxDevice } from "./pkg/uhd_pure.js";

const log = document.querySelector("#log");
const deviceControls = document.querySelector("#device-controls");
const registerControls = document.querySelector("#register-controls");
let device;

const report = (message) => {
  log.textContent = String(message);
};

const run = async (operation) => {
  try {
    await operation();
  } catch (error) {
    report(`Error: ${error}`);
  }
};

await init();
report("Ready. WebUSB requires a supporting browser and a secure context (HTTPS or localhost).");

document.querySelector("#connect").addEventListener("click", () => run(async () => {
  device = await B2xxDevice.request();
  if (!device) {
    report("No device selected.");
    return;
  }
  deviceControls.disabled = false;
  report(
    `Connected ${device.productName ?? "B2xx"} ` +
    `${device.vendorId.toString(16).padStart(4, "0")}:` +
    `${device.productId.toString(16).padStart(4, "0")} ` +
    `serial=${device.serialNumber ?? "unknown"}`,
  );
}));

document.querySelector("#probe").addEventListener("click", () => run(async () => {
  const [usb, firmware, state, identity] = await Promise.all([
    device.usbVersion(),
    device.firmwareCompatibility(),
    device.fx3State(),
    device.motherboardIdentity(),
  ]);
  report(`USB ${usb}\nfirmware=${firmware}\nFX3=${state}\n${identity}`);
}));

document.querySelector("#load-fpga").addEventListener("click", () => run(async () => {
  const file = document.querySelector("#fpga").files[0];
  if (!file) throw new Error("Choose a B2xx FPGA .bin file first");
  report(`Loading ${file.name}…`);
  report(`FPGA: ${await device.loadFpga(new Uint8Array(await file.arrayBuffer()), false)}`);
}));

document.querySelector("#open-transport").addEventListener("click", () => run(async () => {
  await device.openTransport();
  registerControls.disabled = false;
  report("FPGA bulk interfaces claimed.");
}));

document.querySelector("#peek").addEventListener("click", () => run(async () => {
  const address = Number(document.querySelector("#address").value);
  const value = await device.peek32(address);
  report(`0x${address.toString(16)} = 0x${value.toString(16).padStart(8, "0")}`);
}));
