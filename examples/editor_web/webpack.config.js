const path = require("path");
const CopyPlugin = require("copy-webpack-plugin");

const dist = path.resolve(__dirname, "dist");

module.exports = {
  devServer: {
  https: true,
  port: 8080, // Choose your preferred port
  host: '0.0.0.0', // Allow access from any IP
  open: true,
  },
  entry: {
    index: "./src/index.js"
  },
  output: {
    path: dist,
    filename: "[name].js"
  },
  plugins: [
    new CopyPlugin({
      patterns: [
        path.resolve(__dirname, "static")
      ],
    }),
  ],
  module: {
    rules: [
      {
        test: /\.js$/,
        exclude: /node_modules/,
        use: {
          loader: 'babel-loader',
          options: {
            presets: ['@babel/preset-env', '@babel/preset-react']
          }
        },
      },
      {
        test: /\.tsx?$/, // 处理 .ts 和 .tsx 文件
        use: 'ts-loader',
        exclude: /node_modules/,
      },
    ],
  },
  resolve: {
    extensions: ['.tsx', '.ts', '.js'], // 支持导入时省略扩展名
  },
  experiments: {
    asyncWebAssembly: true,
  },
  performance: {
    maxAssetSize: 500000,
    maxEntrypointSize: 500000,
  }
};
