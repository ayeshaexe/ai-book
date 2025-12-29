import React from 'react';
import Layout from '@theme/Layout';
import FloatingChatbot from '@site/src/components/FloatingChatbot/FloatingChatbot';

export default function LayoutWithChatbot(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <FloatingChatbot />
    </>
  );
}