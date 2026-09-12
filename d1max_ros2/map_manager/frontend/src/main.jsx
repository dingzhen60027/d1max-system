import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App.jsx'
import { TooltipProvider } from '@/components/ui/tooltip'
import './styles.css'
import './workspace.css'

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <TooltipProvider delay={350}>
      <App />
    </TooltipProvider>
  </React.StrictMode>,
)
