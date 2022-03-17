node-gyp configure
npm run build

if [ $? -eq 0 ]
then
    npm start
fi
