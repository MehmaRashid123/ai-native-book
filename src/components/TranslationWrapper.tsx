import React, { useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Dictionary: English -> Urdu
const DICTIONARY = {
  "Introduction": "Taaruf (تعارف)",
  "Module": "Hissa (حصہ)",
  "Overview": "Jaiza (جائزہ)",
  "Physical AI": "Physical AI (جسمانی مصنوعی ذہانت)",
  "Robotics": "Robotics (روبوٹکس)",
  "Hardware": "Hardware (ہارڈ ویئر)",
  "System": "System (نظام)",
  "Agents": "Agents (ایجنٹس)",
  "Simulation": "Simulation (کمپیوٹر نقل)",
  "Digital Twin": "Digital Twin (ڈیجیٹل جڑواں)",
  "Sensors": "Sensors (سینسر)",
  "Vision": "Vision (بصارت)",
  "Language": "Language (زبان)",
  "Action": "Action (عمل)",
  "Getting Started": "Shuru Karein (شروع کریں)",
  "Prerequisites": "Zarooriat (ضروریات)",
  "Syllabus": "Nisaab (نصاب)"
};

const TranslationWrapper = ({ children }) => {
  // Initialize state from localStorage to persist translation across page loads
  const [isUrdu, setIsUrdu] = useState(() => {
    if (typeof window !== 'undefined') {
      const savedState = localStorage.getItem('translationState');
      return savedState ? JSON.parse(savedState) : false;
    }
    return false;
  });

  // Function to translate page content with AI backend
  const translatePageWithAI = async () => {
    if (!ExecutionEnvironment.canUseDOM) return;

    // Select all elements that contain text to translate, excluding navigation/sidebar
    const elements = document.querySelectorAll('main h1, main h2, main h3, main p, main li, article h1, article h2, article h3, article p, article li');

    // Optimization: Limit to first 20 elements to prevent too many requests
    const elementsToProcess = Array.from(elements).slice(0, 20);

    if (isUrdu) {
      // Translate to Urdu
      for (const element of elementsToProcess) {
        const text = element.textContent?.trim();

        // Skip empty or very short strings
        if (!text || text.length < 3) continue;

        // Skip if element is inside navigation or sidebar
        if (element.closest('.navbar') ||
            element.closest('.menu') ||
            element.closest('.sidebar') ||
            element.closest('[role="banner"]') ||
            element.closest('nav')) {
          continue;
        }

        // Store original text in data attribute
        if (!element.getAttribute('data-original-text')) {
          element.setAttribute('data-original-text', text);
        }

        try {
          // Send text to backend for AI translation
          const response = await fetch('http://localhost:8000/api/translate', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ text: text }),
          });

          if (response.ok) {
            const data = await response.json();
            element.textContent = data.translated_text;
          } else {
            console.error('Translation API error:', response.status);
            // Fallback to dictionary translation if API fails
            let translatedText = text;
            for (const [english, urdu] of Object.entries(DICTIONARY)) {
              const regex = new RegExp(`\\b${english}\\b`, 'gi');
              translatedText = translatedText.replace(regex, urdu);
            }
            element.textContent = translatedText;
          }
        } catch (error) {
          console.error('Translation API error:', error);
          // Fallback to dictionary translation if API fails
          let translatedText = text;
          for (const [english, urdu] of Object.entries(DICTIONARY)) {
            const regex = new RegExp(`\\b${english}\\b`, 'gi');
            translatedText = translatedText.replace(regex, urdu);
          }
          element.textContent = translatedText;
        }
      }
    } else {
      // Revert to original text
      for (const element of elementsToProcess) {
        // Skip if element is inside navigation or sidebar
        if (element.closest('.navbar') ||
            element.closest('.menu') ||
            element.closest('.sidebar') ||
            element.closest('[role="banner"]') ||
            element.closest('nav')) {
          continue;
        }

        const originalText = element.getAttribute('data-original-text');
        if (originalText) {
          element.textContent = originalText;
        }
      }
    }
  };

  // Function to translate text nodes safely using dictionary (fallback for non-selected elements)
  const translatePageWithDictionary = (revert = false) => {
    if (!ExecutionEnvironment.canUseDOM) return;

    // Only target the main content area to avoid breaking Sidebar/Nav
    // Focus on elements not handled by AI translation
    const contentArea = document.querySelector('main') || document.body;

    const walker = document.createTreeWalker(
      contentArea,
      NodeFilter.SHOW_TEXT,
      null
    );

    let node;
    while (node = walker.nextNode()) {
      // Skip if this is inside h1, h2, h3, p, li since they're handled by AI translation
      if (['H1', 'H2', 'H3', 'P', 'LI'].includes(node.parentElement?.tagName)) continue;

      // Skip if element is inside navigation or sidebar
      if (node.parentElement?.closest('.navbar') ||
          node.parentElement?.closest('.menu') ||
          node.parentElement?.closest('.sidebar') ||
          node.parentElement?.closest('[role="banner"]') ||
          node.parentElement?.closest('nav')) {
        continue;
      }

      let text = node.nodeValue;

      if (!text) continue;

      Object.keys(DICTIONARY).forEach(english => {
        const urdu = DICTIONARY[english];

        if (revert) {
          // English wapis laana
          if (text.includes(urdu)) {
            // Remove the Urdu part safely
             node.nodeValue = text.replace(urdu, english);
             // Or simpler: text.split(' (')[0] if structure matches
             // But for safety, let's just use exact replacement if possible
             // Better Revert Logic:
             const escapedUrdu = urdu.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
             const regex = new RegExp(escapedUrdu, 'g');
             node.nodeValue = node.nodeValue.replace(regex, english);
          }
        } else {
          // Urdu mein convert karna
          // CRITICAL FIX: Check if already translated to avoid repetition
          if (text.includes(english) && !text.includes(urdu)) {
            // Match whole word only to avoid partial replacement issues
            const regex = new RegExp(`\\b${english}\\b`, 'g');
            node.nodeValue = text.replace(regex, urdu);
          }
        }
      });
    }
  };

  // Function to toggle translation state
  const toggleTranslation = () => {
    const newState = !isUrdu;
    setIsUrdu(newState);
    // Save the state to localStorage so it persists across page loads
    if (typeof window !== 'undefined') {
      localStorage.setItem('translationState', JSON.stringify(newState));
    }
  };

  useEffect(() => {
    // First translate the page content with AI
    translatePageWithAI();

    // Then translate any remaining text with dictionary as fallback
    translatePageWithDictionary(!isUrdu);
  }, [isUrdu]); // This will run whenever isUrdu changes

  return (
    <>
      <div style={{
        position: 'fixed',
        top: '1rem',
        right: '50%',
        transform: 'translateX(50%)',
        zIndex: 9999,
        background: '#10b981',
        padding: '8px 16px',
        borderRadius: '20px',
        boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
        cursor: 'pointer',
        fontWeight: 'bold',
        color: '#000'
      }}
      onClick={toggleTranslation}
      >
        {isUrdu ? '🇬🇧 English' : '🇵🇰 Translate to Urdu'}
      </div>
      {children}
    </>
  );
};

export default TranslationWrapper;