import type { ExtensionContext, PanelExtensionContext } from "@foxglove/extension";
import { createRoot } from "react-dom/client";
import { App } from "./App";
import { Controller } from "./controller";
import styles from "./styles.css";
export function initPanel(context:PanelExtensionContext) {
  const style=document.createElement("style"); style.textContent=styles;
  const mount=document.createElement("div"); mount.style.cssText="height:100%;width:100%;overflow:hidden;container-type:inline-size";
  context.panelElement.append(style,mount);
  const controller=new Controller(context);const root=createRoot(mount);root.render(<App controller={controller}/>);
  return ()=>{controller.destroy();root.unmount();mount.remove();style.remove();};
}
export function activate(context:ExtensionContext) { context.registerPanel({name:"D1 状态监控",initPanel}); }
