import init, { B2xxDevice } from "./pkg/uhd_pure.js";

const firmwareUrl = new URL("./pkg/usrp_b200_fw.hex", import.meta.url);
const fpgaUrl = new URL("./pkg/usrp_b200_fpga.bin", import.meta.url);

const log = document.querySelector("#log");
const deviceControls = document.querySelector("#device-controls");
const registerControls = document.querySelector("#register-controls");
let device;
let firmwareImage;
let fpgaImage;

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

const disconnected = (message) => {
  device = undefined;
  deviceControls.disabled = true;
  registerControls.disabled = true;
  report(message);
};

const downloadFirmware = async () => {
  if (!firmwareImage) {
    const response = await fetch(firmwareUrl);
    if (!response.ok) {
      throw new Error(
        `Could not download ${firmwareUrl}: HTTP ${response.status}. ` +
        "Copy usrp_b200_fw.hex next to uhd_pure_bg.wasm.",
      );
    }
    firmwareImage = new Uint8Array(await response.arrayBuffer());
  }
  return firmwareImage;
};

const downloadFpga = async () => {
  if (!fpgaImage) {
    const response = await fetch(fpgaUrl);
    if (!response.ok) {
      throw new Error(
        `Could not download ${fpgaUrl}: HTTP ${response.status}. ` +
        "Copy usrp_b200_fpga.bin next to uhd_pure_bg.wasm.",
      );
    }
    fpgaImage = new Uint8Array(await response.arrayBuffer());
  }
  return fpgaImage;
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
    `serial=${device.serialNumber ?? "unknown"} ` +
    `firmware=${device.firmwareLoaded ? "running" : "bootloader"}`,
  );
}));

document.querySelector("#load-firmware").addEventListener("click", () => run(async () => {
  const image = await downloadFirmware();
  if (device.firmwareLoaded) {
    report("Firmware is running; resetting the FX3 to its bootloader…");
    await device.resetFx3();
    disconnected(
      "FX3 reset. Click Connect B2xx, select the bootloader device, then click Load FX3 firmware again.",
    );
    return;
  }

  report(`Loading ${firmwareUrl.pathname.split("/").at(-1)}…`);
  await device.loadFirmware(image);
  disconnected(
    "Firmware started. Click Connect B2xx again and select the re-enumerated B200.",
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
  if (!device.firmwareLoaded) {
    throw new Error("Load FX3 firmware and reconnect the B200 before loading the FPGA");
  }
  const file = document.querySelector("#fpga").files[0];
  const name = file?.name ?? fpgaUrl.pathname.split("/").at(-1);
  const image = file
    ? new Uint8Array(await file.arrayBuffer())
    : await downloadFpga();
  report(`Loading ${name}…`);
  report(`FPGA: ${await device.loadFpga(image, false)}`);
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
