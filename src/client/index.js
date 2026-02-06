import React from 'react';
import { createRoot } from 'react-dom/client';
import App from './components/App';
import './styles/theme.css';
import './styles/shared.css';

const container = document.getElementById('root');
const root = createRoot(container);
root.render(<App />); 
