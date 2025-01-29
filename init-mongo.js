db.createUser({
  user: "root",
  pwd: "root",
  roles: [
    {
      role: "readWrite",
      db: "db",
    },
  ],
});
db.createCollection("locations"); //MongoDB creates the database when you first store data in that database
